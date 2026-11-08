"""Measured TCP clearance and a fingerprint of every runtime task/asset source."""

import hashlib
from pathlib import Path

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix


def tools_clear(objects, tools, sizes, clearance=0.030):
    points = np.asarray(tools, dtype=float)
    if points.shape != (2, 3) or not np.isfinite(points).all():
        return False
    for name, size in sizes.items():
        obj = objects[name]
        q = np.asarray(obj.quat_xyzw, dtype=float)
        if not np.isfinite(obj.pos).all() or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-12:
            return False
        rotation = quat_xyzw_to_matrix(q / np.linalg.norm(q))
        local = (points - obj.pos) @ rotation
        outside = np.maximum(np.abs(local) - np.asarray(size) / 2, 0)
        if np.any(np.linalg.norm(outside, axis=1) < clearance):
            return False
    return True


def source_revision(task_path):
    task_path = Path(task_path)
    package = Path(__file__).parent
    files = [task_path, *sorted(package.glob("*.py")), *sorted(package.glob("*.usda"))]
    digest = hashlib.sha256()
    for path in files:
        digest.update(path.name.encode() + b"\0" + path.read_bytes() + b"\0")
    return digest.hexdigest()
