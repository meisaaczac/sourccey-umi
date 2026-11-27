# Electronics, Software & Web App

## Wiring
`STS3215 servo -> USB bus servo adapter -> PC` with a 7.4 V supply on the servo (it needs power to read its encoder even with torque off). Give the servo its own ID (examples below use `6`; check the Sourccey manual for your arm's default). Plug the Sourccey 20x20 mm USB camera into the same PC.

## Install
```bash
pip install -r requirements.txt
```

## Calibrate (once, and after any reassembly)
```bash
python software/sts_angle.py calibrate --port /dev/ttyACM0 --id 6
```
Pull the trigger closed and press Enter, then release to the open stop and press Enter. The script figures out the sign automatically, maps raw counts to `q` (`-5°` closed to `60°` open), and warns if travel isn't `65° +/- 6°`.

## Option A: Use the Sourccey UMI Studio Web App (Recommended)
Inspired by Pollen Robotics' [`grabette-fleet`](https://huggingface.co/spaces/pollen-robotics/grabette-fleet) and [`grabette-slam`](https://huggingface.co/spaces/pollen-robotics/grabette-slam), adapted for Sourccey's single 20x20 mm camera + STS3215 servo:

```bash
pip install -r space/requirements.txt
python space/app.py    # opens http://localhost:7860
```

- **Tab 1 (Record & Sessions):** create tasks, record directly from your local USB camera + STS3215, delete bad takes, or upload episode `.zip` files when hosted in the cloud.
- **Tab 2 (Build LeRobot Dataset):** runs `software/mono_slam.py` (tabletop ArUco marker for metric scale + ORB visual odometry) -> `software/build_episode.py` (5-DoF Sourccey URDF IK) -> `software/grabette_lerobot.py` (LeRobot dataset + Hugging Face Hub push).
- **Tab 3 (Visualizer):** opens your published dataset right inside [`lerobot/visualize_dataset`](https://huggingface.co/spaces/lerobot/visualize_dataset).
- You can also deploy [`space/`](../space/README.md) straight to Hugging Face as a Docker Space using [`space/Dockerfile`](../space/Dockerfile).

## Option B: Run the CLI Pipeline Directly

1. **Record an episode** (synchronized wrist video + ~200 Hz servo angle):
   ```bash
   python software/record_episode.py --out ep001 --camera 0 --port /dev/ttyACM0 --id 6 --seconds 60
   ```
   Writes `video.mp4`, `frames.csv`, `angle.csv`, and `meta.json` on the same monotonic clock.

2. **Estimate 6-DoF pose from the 20x20 mm wrist camera** (place a printed 80 mm ArUco `4x4_50` ID 10 marker on the table for metric scale):
   ```bash
   python software/mono_slam.py --ep ep001 --out ep001/pose.csv --table-marker-id 10 --marker-mm 80
   ```

3. **Solve 5-DoF Sourccey IK and run QC:**
   ```bash
   python software/build_episode.py --ep ep001 --pose ep001/pose.csv --cfg cfg.json --out ep001/episode.npz
   ```
   An example `cfg.json` lives in `build_episode.CFG_EXAMPLE`. Because the Sourccey arm has 5 joints, the script logs per-frame position/orientation error and marks unreachable frames as invalid.

4. **Export to LeRobot (and optionally push to Hugging Face Hub):**
   ```bash
   python software/grabette_lerobot.py to-lerobot \
     --eps ep001 \
     --out lerobot_sourccey \
     --task "pick up the red block" \
     --repo-id <your-hf-user>/sourccey-umi-demo \
     --push-to-hub
   ```

5. **Optional: Export to Pollen Grabette's raw episode layout** (`raw_video.mp4`, `frame_timestamps.json`, `angle_data.json`, `frames.json`, `metadata.json`):
   ```bash
   python software/grabette_lerobot.py to-grabette --ep ep001 --out grabette_ep001 --hand left
   ```
   `software/grabette_lerobot.py` also includes `StsAngleCapture`, a drop-in adapter for Grabette's `packages/grabette/grabette/hardware/angle.py` that reads Sourccey's STS3215 instead of I2C magnetic encoders.

## Tests
```bash
python software/test_software.py           # fake servo reader + IK round trip
python software/test_pipeline.py           # simulated recorder, video sync, and episode build
python software/test_grabette_lerobot.py   # Grabette angle adapter + Grabette/LeRobot exporters
python software/test_mono_slam.py          # single-camera monocular SLAM (ArUco metric anchor + ORB VO)
```
