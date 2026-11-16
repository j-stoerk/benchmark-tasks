"""Verify actual CPU and CCD configuration before any policy action."""

import json


def verify_cpu_ccd(env, directory=None):
    import omni.usd

    from . import spec

    stage = omni.usd.get_context().get_stage()
    scenes = [prim for prim in stage.Traverse() if prim.GetTypeName() == "PhysicsScene"]
    report = {"device": str(env.device), "scene_ccd": {}, "body_ccd": {}}
    for prim in scenes:
        report["scene_ccd"][str(prim.GetPath())] = prim.GetAttribute("physxScene:enableCCD").Get()
    for part in spec.PARTS:
        prim = stage.GetPrimAtPath("/World/envs/env_0/" + part["name"])
        report["body_ccd"][part["name"]] = prim.GetAttribute("physxRigidBody:enableCCD").Get()
    print("NATIVE_CPU_CCD_AUDIT " + json.dumps(report), flush=True)
    if directory is not None:
        directory.mkdir(parents=True, exist_ok=True)
        (directory / "native-physics-configuration.json").write_text(
            json.dumps(report, indent=2) + "\n", encoding="utf-8"
        )
    if (
        report["device"] != "cpu"
        or not report["scene_ccd"]
        or not all(report["scene_ccd"].values())
        or not all(report["body_ccd"].values())
    ):
        raise RuntimeError("Native CPU/CCD configuration was not applied; no valid task evaluation can start")
