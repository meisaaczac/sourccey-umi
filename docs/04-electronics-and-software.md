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
python software/test_software.py     # reader with a fake servo + IK round trip
python software/test_pipeline.py     # recorder, video sync and episode, all simulated
```

## What is NOT here
- SLAM/pose: use GoPro + the UMI pipeline (MIT), Quest (HandUMI) or Grabette (Apache-2.0), which also exports a LeRobot dataset.
- In Grabette, the recorder reads 2 magnetic sensors over I2C (`packages/grabette/grabette/hardware/angle.py`). To use Grabette as the recorder, replace that read with `StsAngle.read_q()`.
