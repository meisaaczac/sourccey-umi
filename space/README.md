---
title: Sourccey UMI Studio
emoji: 🤖
colorFrom: blue
colorTo: green
sdk: docker
app_port: 7860
---

# Sourccey UMI — Fleet, Monocular SLAM & LeRobot Builder

A self-contained web dashboard and Hugging Face Space modeled after Pollen Robotics' [`grabette-fleet`](https://huggingface.co/spaces/pollen-robotics/grabette-fleet) and [`grabette-slam`](https://huggingface.co/spaces/pollen-robotics/grabette-slam), adapted specifically for **Vulcan Robotics Sourccey UMI** (using **only the 20×20 mm Sourccey wrist camera + Feetech STS3215 servo**, no GoPro or depth camera required).

## Features
1. **Record & Manage Sessions:** Create natural-language tasks, record directly from local USB camera + STS3215 (when run locally), or upload raw episode ZIPs (when hosted on Hugging Face Spaces).
2. **Monocular SLAM + Sourccey 5-DoF IK:** Recovers metric 6-DoF camera trajectories using a printed tabletop ArUco anchor (`cv2.solvePnP`) + ORB visual odometry (`software/mono_slam.py`), then retargets onto the real Sourccey URDF (`software/build_episode.py`).
3. **One-Click LeRobot Export & Visualizer:** Packages episodes into a standard LeRobot dataset (`software/grabette_lerobot.py`), pushes to Hugging Face Hub, and embeds [`lerobot/visualize_dataset`](https://huggingface.co/spaces/lerobot/visualize_dataset).

## Run Locally
```bash
pip install -r space/requirements.txt
python space/app.py
# Open http://localhost:7860
```

## Deploy to Hugging Face Spaces
Create a new **Docker** Space on Hugging Face and push this repository (using `space/Dockerfile` at the root or copying the `space/` + `software/` + `tools/` folders).
