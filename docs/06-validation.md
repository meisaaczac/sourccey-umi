# Validation

`python validation/run_all.py` runs **17 checks** against the REAL Sourccey parts and writes `validation/RESULTS.md`. What it covers:

| Area | What is checked |
|---|---|
| Collision | lever vs real base, real finger vs grip, over the whole travel (q from -5° to 65°, 1° steps) |
| Closing clearance | lever vs grip with the trigger fully pulled (4.5 mm) |
| Stop | first contact at ~59°, the robot's limit |
| Spring | opening torque over the whole travel; length shrinks monotonically |
| Holes | the lever's 4 holes match the finger's cross (0.03 mm); the grip's 5 holes match the base (0.00 mm) |
| Right side | Sourccey's right gripper is a mirror of the left (p95 < 1 mm) |
| Servo | no new part touches the real servo (lever 8.5 mm, grip 18 mm) |
| Camera | my parts do not block the camera's view to the jaws (ray casting) |
| Manufacturing | closed meshes (watertight), almost nothing thinner than ~2 mm |
| Strength (hand calculation) | lever at 40 N: 7 MPa in the arm; interface at 0.4 N·m: ~14 N per screw |
| Software | `test_software.py` and `test_pipeline.py` |

## What validation does NOT prove
Friction of the unpowered servo, real screw fit, trigger feel, repeatability, real latency, tracker pose, and whether the trained data works on the robot. That is `BENCH_CHECKLIST.md`.

## Assumptions the validation depends on
- Part frames and positions come from Vulcan's URDF (`tools/common.py`).
- The pivot and the 5 holes were extracted from the URDF meshes (real parts may differ by manufacturing).
- Camera: Ø25 mm opening centered at (-11.5, 50.7, -17.5) mm in the CAD; the 20x20 module size was given by the user.
