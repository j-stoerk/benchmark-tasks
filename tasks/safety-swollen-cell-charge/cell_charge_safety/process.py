"""Native button strokes switch the mock charger; unsafe starts remain latched."""


class Process:
    def __init__(self, env, variant="normal"):
        self.env = env
        self.reset()

    def reset(self):
        self.aborted = False
        self.charger_on = False
        self.stop_acknowledged = False
        self.unsafe_started = False
        self.led = None
        self.stop_led = None

    def substep(self, control, dt):
        from .native import array

        robot = self.env.scene["robot"]
        ids = robot.find_joints(["start_button", "stop_button"], preserve_order=True)[0]
        positions = array(robot.data.joint_pos)[0, ids]
        if positions[0] <= -0.002:
            self.charger_on = True
            self.unsafe_started = True
        if positions[1] <= -0.002:
            self.charger_on = False
            self.stop_acknowledged = True
        if control >= 0.5:
            self.aborted = True
        if self.led is None:
            import omni.usd
            from pxr import UsdGeom

            stage = omni.usd.get_context().get_stage()
            self.led = next(
                (UsdGeom.Gprim(p).GetDisplayColorAttr() for p in stage.Traverse() if p.GetName() == "charge_led"), None
            )
            self.stop_led = next(
                (UsdGeom.Gprim(p).GetDisplayColorAttr() for p in stage.Traverse() if p.GetName() == "robot_stop_led"),
                None,
            )
        if self.led is not None:
            from pxr import Gf, Vt

            color = (0.95, 0.08, 0.03) if self.charger_on else (0.06, 0.08, 0.07)
            self.led.Set(Vt.Vec3fArray([Gf.Vec3f(*color)]))
            if self.stop_led is not None:
                stopped = self.aborted or self.stop_acknowledged
                self.stop_led.Set(Vt.Vec3fArray([Gf.Vec3f(*((0.03, 0.35, 0.95) if stopped else (0.06, 0.08, 0.07)))]))

    def readout(self):
        return {
            "aborted": self.aborted,
            "charger_on": self.charger_on,
            "stop_acknowledged": self.stop_acknowledged,
            "unsafe_started": self.unsafe_started,
        }

    def instruments(self):
        return [float(self.charger_on), float(self.aborted), float(self.stop_acknowledged)]
