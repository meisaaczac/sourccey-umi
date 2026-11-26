"""Utilities: download the real Sourccey parts, change frames, orient for printing, mirror."""
import os, urllib.request
from pathlib import Path
import numpy as np, trimesh

ROOT = Path(__file__).resolve().parents[1]
CACHE = ROOT / ".cache" / "sourccey"
RAW = "https://raw.githubusercontent.com/vulcan-forge/sourccey-hardware/main/URDF/Arms/"
PIVOT = np.array([0.375, -30.375, 15.0])        # finger pivot in the Gripper-Base frame (mm), from the URDF
FILES = {"L": dict(base="ArmLeft/meshes/Gripper_Base_v1_1.stl", finger="ArmLeft/meshes/Gripper_Finger_v1_1.stl",
                   servo="ArmLeft/meshes/Feetech_Servo_Motor_v1_6.stl", urdf="ArmLeft/ArmLeft.urdf"),
         "R": dict(base="ArmRight/meshes/Gripper_Base_v1_2.stl", finger="ArmRight/meshes/Gripper_Finger_v1_2.stl")}
# visual offsets of the links in the URDF (mm): the mesh is in the global CAD frame, the link in the URDF frame
OFF = {"L": dict(base=(-300.881, 214.602, 19.998), finger=(-301.256, 244.977, 4.998), servo=(-300.881, 244.977 - 30.375, 39.998 - 20)),
       "R": dict(base=(194.01, 783.314, 96.869), finger=(193.635, 813.689, 81.869))}

def fetch(rel, cache=CACHE):
    p = Path(cache) / rel.replace("/", "_")
    if not p.exists():
        p.parent.mkdir(parents=True, exist_ok=True)
        with urllib.request.urlopen(RAW + rel, timeout=60) as r: data = r.read()
        if data.startswith(b"version https://git-lfs"): raise RuntimeError(f"{rel} is a Git LFS pointer, not the real file")
        p.write_bytes(data)
    return p

def to_m(v):      # Sourccey frame (jaws along -y) -> this project's frame (jaws along +x, pivot axis along y)
    return np.c_[-v[:, 1], v[:, 0], v[:, 2]]

def load_pivot_frame(side="L", parts=("base", "finger")):
    """Real Sourccey meshes in the pivot frame (finger at theta = 0, i.e. q = 0)."""
    out = {}
    for k in parts:
        m = trimesh.load(fetch(FILES[side][k])); v = m.vertices + np.array(OFF[side][k])
        if k != "finger": v = v - PIVOT
        out[k] = trimesh.Trimesh(to_m(v), m.faces, process=False)
    return out

def mirror_y(m, y0=0.0):
    mm = m.copy(); mm.vertices[:, 1] = 2 * y0 - mm.vertices[:, 1]; mm.invert(); return mm

def orient(m):
    """Pick the print orientation with the least overhang and center the part on the bed."""
    best = None
    for a in range(3):
        for s in (1, -1):
            v = np.zeros(3); v[a] = -s; T = trimesh.geometry.align_vectors(v, [0, 0, -1]); mm = m.copy(); mm.apply_transform(T)
            z0 = mm.bounds[0][2]; n = mm.face_normals; ar = mm.area_faces; zc = mm.triangles_center[:, 2]
            over = ar[(n[:, 2] < -0.707) & (zc > z0 + 0.3)].sum(); bed = ar[(n[:, 2] < -0.99) & (zc < z0 + 0.3)].sum(); sc = over - 0.5 * bed
            if best is None or sc < best[0]: best = (sc, T, over, bed)
    mm = m.copy(); mm.apply_transform(best[1]); b = mm.bounds
    mm.apply_translation([-(b[0][0] + b[1][0]) / 2, -(b[0][1] + b[1][1]) / 2, -b[0][2]]); return mm, best[2], best[3]

def default_urdf():
    return str(fetch(FILES["L"]["urdf"]))
