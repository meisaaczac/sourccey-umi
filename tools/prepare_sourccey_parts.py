"""Download the REAL Sourccey gripper base and finger (vulcan-forge/sourccey-hardware) and write print-ready STLs to cad/stl_sourccey/.
  python tools/prepare_sourccey_parts.py"""
import sys; from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent)); import common as C
out = C.ROOT / "cad" / "stl_sourccey"; out.mkdir(parents=True, exist_ok=True)
for side, tag in (("L", "LEFT"), ("R", "RIGHT")):
    parts = C.load_pivot_frame(side)
    for k, m in parts.items():
        o, over, bed = C.orient(m); name = f"{k}_sourccey_gripper_{tag}.stl"; o.export(out / name); print(name, (o.bounds[1] - o.bounds[0]).round(0).tolist(), "mm")
print("ready in", out)
