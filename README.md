<div align="center">

# Sourccey UMI

**A handheld Universal Manipulation Interface built around the real [Vulcan Robotics Sourccey](https://vulcanrobotics.ai) gripper.**

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![CAD Checks](https://img.shields.io/badge/CAD%20Validation-17%2F17%20Passed-22c55e.svg)](validation/RESULTS.md)
[![Hardware](https://img.shields.io/badge/Hardware-PETG%20%2B%20STS3215-f97316.svg)](BOM.md)
[![Platform](https://img.shields.io/badge/Robot-Vulcan%20Sourccey-1e293b.svg)](https://github.com/vulcan-forge/sourccey-hardware)

<p>
  <a href="#how-it-works">How It Works</a> ·
  <a href="#demo-human-umi--sourccey-robot">Demo Video</a> ·
  <a href="docs/03-assembly.md">Assembly Guide</a> ·
  <a href="BOM.md">Bill of Materials</a> ·
  <a href="docs/04-electronics-and-software.md">Software & IK</a> ·
  <a href="validation/RESULTS.md">Validation</a>
</p>

![Sourccey UMI Hero Render](docs/img/00_hero.png)

</div>

---

## Why Sourccey UMI?

Most handheld UMI grippers suffer from an **embodiment gap**: the handheld jaws, compliance, and camera viewpoint differ from the target robot. **Sourccey UMI eliminates that gap by using the Sourccey's actual gripper:**

- **Same gripper hardware** — Bolts directly onto the real Sourccey base and hinged finger (`vulcan-forge/sourccey-hardware`).
- **Same servo encoder** — Uses the Sourccey's own Feetech STS3215 (torque off, read-only) at **0.088° resolution**, so the measured angle *is* the robot's `gripper` joint (`-5°` closed to `60°` open).
- **Single-camera, low-cost design** — Uses **only the Sourccey's 20 × 20 mm wrist camera** behind the base's Ø25 mm `+y` side wing (zero occlusion, ray-cast verified). **No GoPro required.**
- **Two printable add-ons only** — Just a parametric **pistol grip** (5-hole wrist interface + optional 1/4-20 accessory mount) and a **trigger lever** (4-hole finger cross + return spring + velcro strap). Both LEFT and RIGHT arm variants included.
- **Grabette-inspired Web App & HF Space ([`space/`](space/README.md))** — Includes a ready-to-run Gradio dashboard + Docker Hugging Face Space modeled after Pollen Robotics' `grabette-fleet` & `grabette-slam`, adapted for Sourccey's single 20×20 mm RGB camera (`software/mono_slam.py` → 5-DoF IK → LeRobot dataset → embedded [`lerobot/visualize_dataset`](https://huggingface.co/spaces/lerobot/visualize_dataset)).

---

## Demo: Human UMI → Sourccey Robot

A human demonstrates a tabletop pick-and-place with the handheld Sourccey UMI (Phase 1); the recorded 6-DoF trajectory and STS3215 gripper opening (`q`) are retargeted via 5-DoF IK onto the real Sourccey arm URDF (Phase 2). Notice the **identical wrist-camera viewpoint** in the top-right PiP.

![Sourccey UMI to Sourccey Robot Demo](docs/media/sourccey_umi_demo.gif)

> **Full HD render (1280×720, 30 fps):** [`docs/media/sourccey_umi_demo.mp4`](docs/media/sourccey_umi_demo.mp4) · Regenerate locally with `python docs/render_demo_video.py`.

---

## How It Works

<table>
  <tr>
    <td width="52%">
      <img src="docs/img/08_how_to_hold.png" alt="Ergonomic grip and trigger operation"/>
    </td>
    <td width="48%">
      <ol>
        <li><b>Pistol grip</b> bolts onto the Sourccey base through the exact 5-hole wrist interface (with 5× M3 heat-set inserts).</li>
        <li><b>Trigger lever</b> bolts onto the 4-hole cross on the outer face of the Sourccey finger.</li>
        <li><b>Pulling the index trigger</b> closes the hinged finger (<code>q = 60° → -5°</code>, ~47 mm pad travel); releasing lets the extension spring (plus velcro finger strap) reopen it cleanly to the 60° mechanical stop.</li>
        <li><b>STS3215 servo</b> (torque off, 7.4 V powered) streams angle at ~200 Hz on the same monotonic clock as the <b>20 × 20 mm Sourccey wrist camera</b>—no GoPro needed.</li>
      </ol>
    </td>
  </tr>
</table>

```mermaid
flowchart LR
    A["Human Hand + Sourccey UMI (20x20 Cam + STS3215)"] -->|"200 Hz q + Wrist Video"| B["record_episode.py"]
    B -->|"5-DoF Sourccey URDF IK"| C["build_episode.py"]
    C -->|"grabette_lerobot.py"| D["LeRobot Dataset / HF Hub Visualizer"]
    D --> E["Sourccey Robot Policy Training & Replay"]
```

---

## Project Status

> [!NOTE]
> Transparent engineering status — read before printing or collecting datasets.

| Subsystem | Status | Details |
|---|---|---|
| **CAD vs. real Sourccey STLs** | **17 / 17 checks pass** | Watertight meshes, clearances, camera ray-cast, hole alignment ([`validation/RESULTS.md`](validation/RESULTS.md)) |
| **Software & LeRobot bridge** | **Simulated tests pass** | Servo reader, recorder, ArUco sync, 5-DoF IK, Grabette & LeRobot exporter (`test_*.py`) |
| **Physical print & bench test** | **Not yet validated** | Complete [`BENCH_CHECKLIST.md`](BENCH_CHECKLIST.md) before training data collection |
| **6-DoF Pose / SLAM** | **External / Optional** | Single 20×20 mm RGB camera built in (no GoPro); for Pollen's [`grabette-slam`](https://huggingface.co/spaces/pollen-robotics/grabette-slam) cloud RGB-D SLAM, attach an optional depth camera (OAK-D SR / Gemini 305) to the top 1/4-20 mount |

---

## Quick Start

```bash
git clone https://github.com/meisaaczac/sourccey-umi && cd sourccey-umi
pip install -r requirements.txt

# 1. Download & orient the REAL Sourccey base and finger STLs from Vulcan Robotics
python tools/prepare_sourccey_parts.py

# 2. (Optional) Regenerate parametric trigger lever & pistol grip (LEFT & RIGHT)
python cad/umi_cad.py

# 4. Launch the Sourccey UMI Studio web app (Record -> Monocular SLAM -> LeRobot -> Visualizer)
pip install -r space/requirements.txt
python space/app.py                      # http://localhost:7860 (or deploy space/ as a HF Docker Space)
```

Then follow the build & setup workflow:
1. **[Printing Guide](docs/02-printing.md)** — PETG settings, orientation, and LEFT/RIGHT files
2. **[Illustrated Assembly Guide](docs/03-assembly.md)** — 9-step build with checklists, exploded view, and troubleshooting
3. **[Electronics & Software](docs/04-electronics-and-software.md)** — STS3215 wiring, calibration, recording, ArUco sync, and IK
4. **[Data Collection Protocol](docs/05-data-collection.md)** — Best practices for the 5-DoF Sourccey arm
5. **[Bench Checklist](BENCH_CHECKLIST.md)** — Physical verification gates before dataset recording

---

## Key Specifications

| Parameter | Value |
|---|---|
| **Printed mass** *(4 parts, PETG 40% infill)* | ~187 g *(+ ~55 g STS3215 servo, camera & fasteners)* |
| **Joint range (`q`)** | `-5°` (closed) to `60°` (open stop) |
| **Angle resolution** | `0.088°` per encoder tick (STS3215 12-bit) |
| **Trigger pad travel** | ~47 mm over full 65° sweep |
| **Return spring travel** | 28 mm (open) → 59 mm (closed) |
| **Jaw gap** *(at 55 mm grasp depth)* | `10.6 mm` (`-5°`) · `35 mm` (`20°`) · `57 mm` (`40°`) · `82 mm` (`55°`) |
| **Camera** | Single **Sourccey 20 × 20 mm** wrist camera in the `+y` wing (Ø25 mm opening, no GoPro) |
| **Optional top mount** | Standard 1/4-20 hex slot at `(-51, -7, 34) mm` in pivot frame (leave empty or mount optional RGB-D/tracker) |

---

## Repository Structure

```text
sourccey-umi/
├── BOM.md & bom/bom.csv       # Complete bill of materials (Markdown + CSV)
├── BENCH_CHECKLIST.md         # Physical bench validation tests & pass criteria
├── cad/
│   ├── umi_cad.py             # Parametric CadQuery source for grip & trigger lever
│   ├── stl/                   # Print-ready STLs (LEFT and RIGHT variants)
│   └── step/                  # Editable STEP assemblies
├── docs/
│   ├── 01-how-it-works.md     # Mechanical design & kinematics
│   ├── 02-printing.md         # Slicer settings & part preparation
│   ├── 03-assembly.md         # Illustrated 9-step assembly manual
│   ├── 04-electronics-and-software.md
│   ├── 05-data-collection.md
│   ├── 06-validation.md
│   ├── 07-research-and-decisions.md
│   ├── img/                   # Studio renders & CAD step diagrams
│   ├── media/                 # MuJoCo UMI -> Sourccey robot demo video & GIF
│   ├── make_images.py         # Rebuilds CAD step diagrams
│   └── render_demo_video.py   # Rebuilds MuJoCo pick-and-place video & GIF
├── software/                  # STS3215 reader, recorder, mono SLAM, 5-DoF IK, Grabette/LeRobot bridge
├── space/                     # Sourccey UMI Studio (HF Docker Space + local web UI for Record -> LeRobot)
├── tools/                     # Downloads real Vulcan Sourccey STLs (not redistributed)
└── validation/                # 17 automated geometric/mechanical/software checks
```

---

## Important Limitations

- **5-DoF arm kinematics:** The Sourccey arm has 5 arm joints + gripper, so not every 6-DoF hand pose is reachable. `software/build_episode.py` reports per-frame position/orientation IK errors and filters invalid frames automatically.
- **Unpowered servo friction:** STS3215 gearbox drag can oppose the return spring; keep the **velcro index-finger strap** installed until bench tests 4 & 5 pass.
- **6-DoF SLAM & Pollen `grabette-slam`:** `software/grabette_lerobot.py` exports directly to LeRobot datasets and Grabette's episode format. Note that Pollen Robotics' cloud [`grabette-slam`](https://huggingface.co/spaces/pollen-robotics/grabette-slam) Space runs RTAB-Map **RGB-D** SLAM, which requires a depth stream (e.g., OAK-D SR or Orbbec Gemini 305) if you want cloud RGB-D trajectory reconstruction.

---

## Authors & License

Created by **Isaac de Souza** ([@meisaaczac](https://github.com/meisaaczac)) with AI engineering assistance.
Code, CAD, and documentation are released under the **MIT License** ([`LICENSE`](LICENSE)). Real Vulcan Robotics / Sourccey hardware meshes are downloaded directly from upstream and are **not** redistributed here—see [`NOTICE.md`](NOTICE.md) and [`AUTHORS.md`](AUTHORS.md).
