# Policy Training & Evaluation (ACT / Diffusion Policy)

Once you export your Sourccey UMI episodes into a LeRobot dataset (using `space/app.py` or `software/grabette_lerobot.py`), you can train an **ACT** or **Diffusion Policy** directly with Hugging Face [LeRobot](https://github.com/huggingface/lerobot).

## Install LeRobot
```bash
pip install lerobot
```

## Train ACT (recommended starting point)
```bash
python policy/train_lerobot.py \
  --dataset <your-hf-user>/sourccey-umi-demo \
  --policy act \
  --steps 20000 \
  --output-dir outputs/sourccey_act
```

## Train Diffusion Policy
```bash
python policy/train_lerobot.py \
  --dataset <your-hf-user>/sourccey-umi-demo \
  --policy diffusion \
  --steps 50000 \
  --output-dir outputs/sourccey_diffusion
```

Pass `--dry-run` first to inspect the exact `lerobot-train` command before launching GPU training.

## Tips for Sourccey UMI Policies
- **Latency matching:** when exporting with `grabette_lerobot.py to-lerobot`, try `--action-latency-ms 60` (2 frames at 30 fps) to compensate camera capture + model inference + STS3215 execution lag on the real robot.
- **Relative EE actions:** `grabette_lerobot.py` automatically writes `action.relative_ee` (`[dx, dy, dz, drx, dry, drz, gripper]`) whenever `episode.npz` contains `tool_pose`, making trained policies much more robust to table/robot placement changes.
- **Quality gate:** always pass `--enforce-qc` (or enable the QC gate checkbox in `space/app.py`) so episodes with SLAM jumps or high 5-DoF IK error are excluded before training.
- **Replay before real hardware:** test your trained checkpoint at low speed in the Sourccey MuJoCo simulation (`docs/render_demo_video.py` loads the exact `ArmLeft.urdf` meshes) before deploying on the physical Sourccey arm.
