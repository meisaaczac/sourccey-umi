# Printing

Print **4 parts** in PETG (0.2 mm layers, 0.4 mm nozzle, 4 perimeters, 40% infill; 100% in the lever arms if you can). The STLs are already oriented for printing.

| Part | File | Source | Supports |
|---|---|---|---|
| Gripper base | `stl_sourccey/base_sourccey_gripper_LEFT.stl` | included (from Vulcan Robotics; refresh with `tools/prepare_sourccey_parts.py`) | may need some |
| Finger | `stl_sourccey/finger_sourccey_gripper_LEFT.stl` | same | little |
| Pistol grip | `cad/stl/pistol_grip_LEFT.stl` | this repo | no |
| Trigger lever | `cad/stl/trigger_lever_LEFT.stl` | this repo | no |

**Right arm:** use the `_RIGHT` files (my parts are mirrored; Sourccey's right gripper differs from the mirrored left by about 1 mm at the worst point, p95 0.8 mm, checked).

Tips:
- Test the 4.2 mm insert holes and the 3 mm holes in the finger and base with an M3 screw before assembling everything.
- The lever prints lying flat (layers in the plate's plane), which is the strongest orientation for the trigger load.
- If your printer shrinks parts, print a test piece with only the 5-hole cross face and adjust.
- The Sourccey base and finger STLs in `stl_sourccey/` are already oriented for printing (you can also re-download fresh copies from Vulcan Robotics with `python tools/prepare_sourccey_parts.py`).
