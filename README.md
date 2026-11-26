# Sourccey UMI

**A handheld data collector that uses the Sourccey's REAL gripper.** You hold it, pull the trigger, the gripper closes exactly like the robot's, and the servo (the same STS3215 the Sourccey uses) measures the angle. That angle *is* the robot's `gripper` joint, so the recorded data can teach the Sourccey directly.

![operation](docs/img/09_operation.png)

## Status (read this first)
| | |
|---|---|
| CAD against the real Sourccey parts | **17 of 17 checks pass** (`validation/RESULTS.md`) |
| Software (reader, recorder, sync, IK) | tested in simulation, no hardware |
| Printed and assembled? | **not yet** |
| Hand pose (tracker/SLAM) | **not included**, only the mounting point |
| Ready to collect training data? | **no**: bench validation is still missing (`BENCH_CHECKLIST.md`) |

## How it works in 4 lines
1. The base and finger are the Sourccey's. This repo adds a **pistol grip** (bolts onto the same interface the robot's wrist uses) and a **trigger lever** (bolts onto the 4-hole cross the finger already has).
2. Pulling the trigger with the index finger closes the gripper; releasing it reopens (spring + velcro strap).
3. The servo, with torque off, reads the angle (0.088° per step).
4. The Sourccey's own camera (20x20 mm) sits in the base's side wing, with the same viewpoint as the robot.

## Getting started
```bash
git clone https://github.com/meisaaczac/sourccey-umi && cd sourccey-umi
pip install -r requirements.txt
python tools/prepare_sourccey_parts.py   # downloads the REAL Sourccey base and finger and writes print-ready STLs
python cad/umi_cad.py                    # (optional) regenerate the lever and grip
python validation/run_all.py             # runs the 17 validations
```
Then: **[docs/02-printing.md](docs/02-printing.md)** → **[docs/03-assembly.md](docs/03-assembly.md)** (illustrated step by step) → **[docs/04-electronics-and-software.md](docs/04-electronics-and-software.md)**.

## What is in here
```
BOM.md, bom/bom.csv        bill of materials
docs/                      how it works, printing, assembly (illustrated), electronics, data collection, validation, research
cad/                       parametric CAD (CadQuery), STL and STEP of the new parts (LEFT and RIGHT)
tools/                     downloads and prepares the real Sourccey parts (we do not redistribute them)
software/                  servo reader, recorder, sync, episode builder with IK, tests
validation/                17 automatic checks and RESULTS.md
BENCH_CHECKLIST.md         physical tests still missing, with pass criteria
```

## Important limitations
- The Sourccey arm has **5 joints** + gripper: not every 6D pose recorded by the UMI is reachable (the converter reports the error per frame).
- **Friction of the unpowered servo** may be high; that is why the velcro strap is required until the bench test passes.
- This repo ships no SLAM or LeRobot export: reuse [Grabette](https://github.com/pollen-robotics/grabette) (Apache-2.0).

## Authors and license
Isaac de Souza ([@meisaaczac](https://github.com/meisaaczac)) and Claude (Anthropic). Code and CAD under MIT (`LICENSE`). Vulcan/Sourccey parts and other projects: see `NOTICE.md`.
