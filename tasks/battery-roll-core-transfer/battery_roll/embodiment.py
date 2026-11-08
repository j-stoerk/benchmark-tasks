"""Native YAM simulation with measured release and optional handling-envelope history."""

import json
from dataclasses import replace
from pathlib import Path

import numpy as np
from yam_arena import task_file
from yam_arena.constants import TCP_OFFSET
from yam_arena.inspect_embodiment import YamArenaEmbodiment
from yam_arena.urdf import quat_xyzw_to_matrix

from .release import source_revision, tools_clear


def array(tensor):
    return getattr(tensor, "torch", tensor).detach().cpu().numpy().astype(float)


class BatteryRollEmbodiment(YamArenaEmbodiment):
    """Withhold all credit until the measured grippers release every workpiece."""

    def __init__(self, *, task=None, trace_file=None, **kwargs):
        expected = Path(__file__).resolve().parents[1] / "task.py"
        selected = expected if task is None else Path(task).expanduser().resolve()
        if selected != expected:
            raise ValueError("This embodiment evaluates only its bundled task.py")
        super().__init__(task=str(selected), **kwargs)
        original = self.task
        self._tools = np.full((2, 3), np.nan)
        self._handling_violation = False
        self._trace_file = Path(trace_file).expanduser().resolve() if trace_file else None
        if self._trace_file is not None:
            self._trace_file.parent.mkdir(parents=True, exist_ok=True)
        sizes = original.module.SIZES

        def allowed(objects):
            return not self._handling_violation and tools_clear(objects, self._tools, sizes)

        self.task = replace(
            original,
            success=lambda objects: allowed(objects) and original.success(objects),
            score=lambda objects: original.score(objects) if allowed(objects) else 0.0,
        )
        self.info = replace(
            self.info,
            environment_revision=source_revision(selected)[:12],
            docs=self.info.docs + "\nAll score tiers require both measured TCPs at least 30 mm "
            "outside every movable workpiece. Invalid handling is reported in step info.\n",
        )
        task_file.register(self.info.environment_id, self.task)

    def _ensure_sim(self, goal):
        # Start the same shared Kit app as the native embodiment, then register
        # our assets before the parent's normal scene construction. Physics is
        # built and stepped by the unchanged parent implementation.
        import atexit
        import os

        from yam_arena import inspect_embodiment as native

        if native._APP is None:
            from isaaclab.app import AppLauncher

            os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
            options = {"headless": self.headless, "device": self.device, "enable_cameras": True}
            if not self.headless:
                options["visualizer"] = ["kit"]
                os.environ["DISPLAY"] = native.window_display(os.environ)
            native._APP = AppLauncher(**options).app
            atexit.register(native._close_app_at_exit)
        from .assets import register_assets

        register_assets()
        return super()._ensure_sim(goal)

    def reset(self, scene, *, seed=None):
        self._handling_violation = False
        self._tools = np.full((2, 3), np.nan)
        return super().reset(scene, seed=seed)

    def _task_step(self):
        origin = array(self._base.scene.env_origins[0])
        tools = []
        for side in ("left", "right"):
            robot = self._base.scene[f"{side}_robot"]
            index = robot.body_names.index("gripper")
            pos = array(robot.data.body_pos_w)[0, index] - origin
            quat = array(robot.data.body_quat_w)[0, index]
            tools.append(pos + quat_xyzw_to_matrix(quat) @ np.asarray(TCP_OFFSET))
        self._tools = np.asarray(tools)
        from yam_arena.scene import object_states

        states = object_states(self._env, self.task.names)
        self._handling_violation |= bool(self.task.module.handling_violation(states, self._tools))
        result = super()._task_step()
        readout = {
            name: {
                "pos": states[name].pos.tolist(),
                "quat_xyzw": states[name].quat_xyzw.tolist(),
                "lin_vel": states[name].lin_vel.tolist(),
                "ang_vel": states[name].ang_vel.tolist(),
            }
            for name in self.task.module.SIZES
        }
        readout.update(
            tcp=self._tools.tolist(),
            threaded=bool(self.task.module.threaded(states)),
            on_rests=bool(self.task.module.on_rests(states)),
            suspended=bool(self.task.module.suspended(states)),
            released=tools_clear(states, self._tools, self.task.module.SIZES),
        )
        updated = replace(
            result, info={**result.info, "handling_violation": self._handling_violation, "physics_readout": readout}
        )
        if self._trace_file is not None:
            with self._trace_file.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps({"t": self._t, "info": updated.info}) + "\n")
        return updated


def battery_roll(**kwargs):
    return BatteryRollEmbodiment(**kwargs)
