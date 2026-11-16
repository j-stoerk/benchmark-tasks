"""Read-only native cooked-SDF preflight; never a policy observation or score."""

import json


def verify_cooked_bores(directory=None):
    import omni.physics.tensors as tensors
    import omni.usd
    import torch
    from pxr import UsdPhysics

    from . import spec

    paths = [f"/World/envs/env_0/{part['name']}/tooling_surface" for part in spec.PARTS]
    stage = omni.usd.get_context().get_stage()
    points = [[0, 0, 0], [0.0324, 0, 0], [0.036, 0, 0], [0.060, 0, 0]]
    report = {"query_frame": "mesh local", "points_m": points, "usd": []}
    for path in paths:
        prim = stage.GetPrimAtPath(path)
        report["usd"].append(
            {
                "path": path,
                "schemas": prim.GetAppliedSchemas(),
                "approximation": UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get(),
                "resolution": prim.GetAttribute("physxSDFMeshCollision:sdfResolution").Get(),
            }
        )
    try:
        sim_view = tensors.create_simulation_view("torch")
        sim_view.set_subspace_roots("/")
        view = sim_view.create_sdf_shape_view(paths, len(points))
        report["native_shape_count"] = view.count
        report["native_shape_paths"] = list(view.object_paths)
        if view.count != len(paths):
            raise RuntimeError("Native cooked SDF count differs from nine hollow tooling rings")
        query = torch.tensor([points] * view.count, device="cuda:0", dtype=torch.float32)
        values = view.get_sdf_and_gradients(query).detach().cpu().numpy()
        report["sdf_and_gradients"] = values.tolist()
        distances = values[:, :, 0]
        valid = (
            (distances[:, 0] > 0.030).all()
            and (distances[:, 1] > 0).all()
            and (distances[:, 2] < 0).all()
            and (distances[:, 3] > 0).all()
        )
        report["hollow_clearance_checks_passed"] = bool(valid)
        if not valid:
            raise RuntimeError(
                "Native cooked SDF does not preserve the specified open bore, shaft clearance or tooling material"
            )
    except Exception as error:
        report["error"] = f"{type(error).__name__}: {error}"
        raise
    finally:
        print("NATIVE_COOKED_COLLIDER_AUDIT " + json.dumps(report), flush=True)
        if directory is not None:
            directory.mkdir(parents=True, exist_ok=True)
            (directory / "native-collider-audit.json").write_text(
                json.dumps(report, indent=2) + "\n", encoding="utf-8"
            )
