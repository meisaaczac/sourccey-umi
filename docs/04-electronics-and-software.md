# Electronics and software

## Wiring
`STS3215 servo → USB adapter for Feetech servos → PC`, and a 7.4 V supply on the servo (it needs power to read its encoder even with torque off). Give the servo its own ID (the examples use 6; the Sourccey manual has the default).

## Install
```bash
pip install -r requirements.txt
```

## Calibrate (once, and after any reassembly)
```bash
python software/sts_angle.py calibrate --port /dev/ttyACM0 --id 6
```
Squeeze the trigger until it closes and hold; release to the stop. The script finds the direction by itself, computes `q` (degrees of the `gripper` joint: -5 closed, 60 open) and warns if the measured travel is not 65° ± 6°.

## Record an episode
```bash
python software/record_episode.py --out ep001 --camera 0 --port /dev/ttyACM0 --id 6 --seconds 60
```
Writes `video.mp4`, `frames.csv`, `angle.csv` and `meta.json` on the same monotonic clock (~200 Hz angle).

## Sync with the tracker video
Stick on the markers `software/aruco_id0_*.png` (finger) and `aruco_id1_*.png` (fixed jaw). At the start of each recording do 3 slow open/close cycles (q 10° to 55°) in view of the tracker:
```python
from sync_offset import video_opening_signal, estimate_lag
```
It compares the distance between the markers in the video with the servo `q` and returns the delay (tested in simulation: it recovered an injected 150 ms with correlation 1.00).

## Build the episode and convert for the robot
```bash
python software/build_episode.py --ep ep001 --pose pose.csv --cfg cfg.json --out ep001/episode.npz
```
Aligns the clocks, resamples, solves IK on the real Sourccey URDF and flags good frames (`valid`). An example `cfg.json` is in `build_episode.CFG_EXAMPLE`. UMI SLAM output goes in through `load_umi_slam()`.
**The arm has 5 joints**: the script reports position and orientation error per frame and invalidates the bad ones.

## Tests
```bash
python software/test_software.py           # reader with a fake servo + IK round trip
python software/test_pipeline.py           # recorder, video sync and episode, all simulated
python software/test_grabette_lerobot.py   # Grabette angle adapter + Grabette/LeRobot exporters
```

## Pollen Robotics Grabette & LeRobot Integration
This repo ships **`software/grabette_lerobot.py`** to connect Sourccey UMI directly to the [Pollen Robotics Grabette](https://github.com/pollen-robotics/grabette) and [Hugging Face LeRobot](https://github.com/huggingface/lerobot) ecosystem—**no GoPro required**:

1. **Export directly to a LeRobot dataset** (and optionally push to Hugging Face Hub to inspect in the [LeRobot Visualizer](https://huggingface.co/spaces/lerobot/visualize_dataset)):
   ```bash
   python software/grabette_lerobot.py to-lerobot \
     --eps ep001 ep002 \
     --out lerobot_sourccey \
     --task "pick up the red block" \
     --repo-id <your-hf-user>/sourccey-umi-demo \
     --push-to-hub
   ```
   - Uses `episode.npz` (full 5-DoF `arm_q` + `gripper`) when present, or exports synchronized wrist-camera video + real STS3215 `gripper` angle directly from a raw `record_episode.py` folder.
   - Writes standard Parquet (if `pyarrow` is installed, else CSV) + MP4 + `meta/info.json`, `episodes.jsonl`, `tasks.jsonl`.

2. **Convert to Grabette's canonical episode layout** (`raw_video.mp4`, `frame_timestamps.json`, `angle_data.json`, `frames.json`, `metadata.json`):
   ```bash
   python software/grabette_lerobot.py to-grabette --ep ep001 --out grabette_ep001 --hand left
   ```
   - Also provides `StsAngleCapture`, a drop-in replacement for Grabette's `packages/grabette/grabette/hardware/angle.py` that maps Sourccey's STS3215 (`60°` open → `-5°` closed) to Grabette's radian convention (`0` open → `+` closing).

> **Note on Pollen's cloud SLAM Space ([`pollen-robotics/grabette-slam`](https://huggingface.co/spaces/pollen-robotics/grabette-slam)):**
> The Grabette Fleet + SLAM Hugging Face Spaces run RTAB-Map **RGB-D** visual-inertial SLAM (`offline_vslam`), which requires depth frames (`dcam_depth.mkv` + `dcam_left.mp4` from an OAK-D SR or Orbbec Gemini 305). With **only the single 20 × 20 mm Sourccey RGB camera**, use `grabette_lerobot.py to-lerobot` directly (paired with monocular SLAM or external pose tracking if you need 6-DoF arm IK).
