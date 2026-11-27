"""Grabette & LeRobot bridge for Sourccey UMI.

Connects Sourccey UMI (20x20 mm Sourccey wrist camera + Feetech STS3215 servo) to:
  1. Pollen Robotics Grabette (`pollen-robotics/grabette`): drop-in STS3215
     `StsAngleCapture` adapter and Grabette episode format exporter (`raw_video.mp4`,
     `frame_timestamps.json`, `angle_data.json`, `frames.json`, `metadata.json`).
  2. Hugging Face LeRobot (`huggingface/lerobot`): direct dataset exporter
     (Parquet/CSV + MP4 + meta/*.json) with optional `--push-to-hub` so you can
     inspect episodes directly in the LeRobot Visualizer web app:
     https://huggingface.co/spaces/lerobot/visualize_dataset

Usage:
  # Convert a sourccey-umi episode into Grabette's canonical episode folder
  python software/grabette_lerobot.py to-grabette --ep ep001 --out grabette_ep001

  # Export one or more episodes into a LeRobot dataset (optionally pushing to HF Hub)
  python software/grabette_lerobot.py to-lerobot --eps ep001 ep002 --out lerobot_ds --task "pick up the red block"
  python software/grabette_lerobot.py to-lerobot --eps ep001 --out lerobot_ds --task "pick block" --repo-id user/sourccey-demo --push-to-hub
"""
import argparse
import csv
import json
import math
import shutil
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np


# ---------------------------------------------------------------------------
# 1. Grabette-compatible AngleCapture adapter backed by Sourccey STS3215
# ---------------------------------------------------------------------------
class StsAngleCapture:
    """Drop-in replacement for `grabette.hardware.angle.AngleCapture`.

    Grabette's native recorder reads two AS5600L I2C magnetic encoders and writes
    `angle_data.json` with `{"samples": [{"cts": ts_ms, "value": [rad, rad]}]}`
    where 0 = fully open and positive = closing.

    This adapter reads Sourccey's single STS3215 servo via `StsAngle` (where
    q = +60 deg is open and q = -5 deg is closed) and maps it cleanly into
    Grabette's convention (`closing_rad = radians(60.0 - q_deg)`).
    """

    def __init__(self, sts_reader, sync_manager=None, sample_rate_hz: float = 100.0):
        self.sts = sts_reader
        self.sync = sync_manager
        self.sample_rate_hz = float(sample_rate_hz)
        self.samples: list[dict] = []
        self._running = False
        self._thread: threading.Thread | None = None
        self._t0_ns = 0

    def _timestamp_ms(self) -> float:
        if self.sync is not None and hasattr(self.sync, "get_timestamp_ms"):
            return float(self.sync.get_timestamp_ms())
        return (time.monotonic_ns() - self._t0_ns) / 1e6

    @staticmethod
    def q_deg_to_grabette_rad(q_deg: float) -> float:
        """Convert Sourccey q_deg (+60 open, -5 closed) to Grabette rad (0 open, + closing)."""
        return math.radians(60.0 - q_deg)

    def _loop(self) -> None:
        dt = 1.0 / self.sample_rate_hz
        nxt = time.monotonic()
        while self._running:
            try:
                q_deg = self.sts.read_q()
                val = self.q_deg_to_grabette_rad(q_deg)
                self.samples.append({"cts": round(self._timestamp_ms(), 3), "value": [val, val]})
            except Exception:
                pass
            nxt += dt
            time.sleep(max(0.0, nxt - time.monotonic()))

    def start_capture(self) -> None:
        if self._running:
            raise RuntimeError("StsAngleCapture already running")
        self.samples = []
        self._t0_ns = time.monotonic_ns()
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> list[dict]:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        return self.samples


# ---------------------------------------------------------------------------
# 2. Export sourccey-umi episode -> Grabette canonical episode layout
# ---------------------------------------------------------------------------
def _load_sourccey_episode(ep_dir: Path):
    meta = json.loads((ep_dir / "meta.json").read_text(encoding="utf-8"))
    t0_ns = int(meta["t_start_monotonic_ns"])

    frames_t_s = []
    with open(ep_dir / "frames.csv", newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            frames_t_s.append((int(row["t_monotonic_ns"]) - t0_ns) / 1e9)

    angle_t_s, angle_q_deg = [], []
    with open(ep_dir / "angle.csv", newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            angle_t_s.append((int(row["t_monotonic_ns"]) - t0_ns) / 1e9)
            angle_q_deg.append(float(row["q_deg"]))

    return meta, np.asarray(frames_t_s, float), np.asarray(angle_t_s, float), np.asarray(angle_q_deg, float)


def export_grabette_episode(ep_dir: str | Path, out_dir: str | Path, hand: str = "left") -> dict:
    """Convert a sourccey-umi episode into Pollen Robotics Grabette's episode layout.

    Produces:
      - raw_video.mp4          (Sourccey 20x20 mm wrist camera)
      - frame_timestamps.json  (capture timestamps in ms on shared monotonic clock)
      - angle_data.json        (STS3215 gripper opening in Grabette's rad convention)
      - frames.json            (Sourccey pivot <-> 20x20 mm wrist camera transform)
      - metadata.json          (duration, counts, hand, camera & hardware info)

    Note: `pollen-robotics/grabette-slam` cloud Space runs RTAB-Map RGB-D SLAM and
    requires an additional depth stream (`dcam_depth.mkv` + `dcam_left.mp4`). With
    the single 20x20 mm Sourccey RGB camera, use `export_lerobot_dataset()` directly
    or pair with monocular/external 6-DoF pose estimation.
    """
    ep_dir, out_dir = Path(ep_dir), Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    meta, f_t, a_t, a_q = _load_sourccey_episode(ep_dir)

    video_src = ep_dir / "video.mp4"
    if video_src.exists():
        shutil.copy2(video_src, out_dir / "raw_video.mp4")

    # frame_timestamps.json: list of ms timestamps on episode clock
    frame_ts_ms = [round(float(t * 1000.0), 3) for t in f_t]
    (out_dir / "frame_timestamps.json").write_text(json.dumps(frame_ts_ms, indent=2), encoding="utf-8")

    # angle_data.json: Grabette schema {"samples": [{"cts": ms, "value": [rad, rad]}]}
    samples = [
        {"cts": round(float(t * 1000.0), 3), "value": [round(StsAngleCapture.q_deg_to_grabette_rad(q), 6)] * 2}
        for t, q in zip(a_t, a_q)
    ]
    (out_dir / "angle_data.json").write_text(json.dumps({"samples": samples}, indent=2), encoding="utf-8")

    # frames.json: Sourccey wrist camera (20x20 mm in +y wing) relative to finger pivot
    c, s = math.cos(math.radians(-22.0)), math.sin(math.radians(-22.0))
    T_cam_in_pivot = [
        [c, -s, 0.0, -0.0115],
        [s, c, 0.0, 0.0507 if hand == "left" else -0.0507],
        [0.0, 0.0, 1.0, -0.0175],
        [0.0, 0.0, 0.0, 1.0],
    ]
    frames_json = {
        "reference_frame": "sourccey_finger_pivot",
        "wrist_camera": T_cam_in_pivot,
        "gripper_center": [[1, 0, 0, 0.062], [0, 1, 0, 0.0], [0, 0, 1, -0.020], [0, 0, 0, 1]],
    }
    (out_dir / "frames.json").write_text(json.dumps(frames_json, indent=2), encoding="utf-8")

    duration_s = float(f_t[-1] - f_t[0]) if len(f_t) > 1 else 0.0
    g_meta = {
        "device": "sourccey-umi",
        "hand": hand,
        "urdf": f"sourccey_arm_{hand}",
        "angle_convention": "0_open_positive_closing_rad",
        "angle_source": "feetech_sts3215",
        "camera": "sourccey_20x20mm_rgb",
        "depth_camera": None,
        "frame_count": len(frame_ts_ms),
        "angle_sample_count": len(samples),
        "duration_s": round(duration_s, 3),
        "exported_utc": datetime.now(timezone.utc).isoformat(),
        "source_meta": meta,
    }
    (out_dir / "metadata.json").write_text(json.dumps(g_meta, indent=2), encoding="utf-8")
    return g_meta


# ---------------------------------------------------------------------------
# 3. Export sourccey-umi episode(s) -> LeRobot dataset (+ optional HF Hub push)
# ---------------------------------------------------------------------------
JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]


def export_lerobot_dataset(
    ep_dirs: list[str | Path],
    out_dir: str | Path,
    task: str = "manipulate object",
    fps: float = 30.0,
    repo_id: str | None = None,
    push_to_hub: bool = False,
) -> dict:
    """Assemble a LeRobot-compatible dataset from one or more sourccey-umi episodes.

    Each episode directory must contain `video.mp4`, `frames.csv`, `angle.csv`,
    `meta.json`. If `episode.npz` (from `build_episode.py` with 5-DoF IK) is also
    present inside the episode directory, full 6-DoF `[arm_q(5), gripper_q_rad]`
    states/actions are exported; otherwise arm joints default to 0.0 while the real
    STS3215 gripper joint (`gripper`) and synchronized wrist camera video are exported.
    """
    out_dir = Path(out_dir)
    meta_dir = out_dir / "meta"
    data_dir = out_dir / "data" / "chunk-000"
    vid_dir = out_dir / "videos" / "chunk-000" / "observation.images.wrist"
    for d in (meta_dir, data_dir, vid_dir):
        d.mkdir(parents=True, exist_ok=True)

    try:
        import pyarrow as pa
        import pyarrow.parquet as pq
        has_parquet = True
    except Exception:
        has_parquet = False

    total_frames = 0
    episodes_meta = []
    video_shape = [720, 1280, 3]

    for ep_idx, ep_path in enumerate(ep_dirs):
        ep_path = Path(ep_path)
        meta, f_t, a_t, a_q_deg = _load_sourccey_episode(ep_path)
        if "height" in meta and "width" in meta:
            video_shape = [int(meta["height"]), int(meta["width"]), 3]

        npz_path = ep_path / "episode.npz"
        if npz_path.exists():
            z = np.load(npz_path)
            t = z["t"]
            arm_q = z["arm_q"]
            gq_rad = np.radians(z["gripper_q_deg"])
        else:
            t = f_t
            arm_q = np.zeros((len(t), 5), dtype=np.float32)
            gq_rad = np.radians(np.interp(t, a_t, a_q_deg)).astype(np.float32)

        n = len(t)
        state = np.hstack([arm_q.astype(np.float32), gq_rad.reshape(-1, 1).astype(np.float32)])
        # Next-step action target (last frame holds final state)
        action = np.vstack([state[1:], state[-1:]]) if n > 1 else state.copy()

        # Copy wrist camera video
        vid_src = ep_path / "video.mp4"
        if vid_src.exists():
            shutil.copy2(vid_src, vid_dir / f"episode_{ep_idx:06d}.mp4")

        # Write episode tabular data (Parquet if pyarrow installed, else CSV)
        timestamps = (t - t[0]).astype(np.float32) if n > 0 else np.zeros(0, np.float32)
        indices = np.arange(total_frames, total_frames + n, dtype=np.int64)
        frame_indices = np.arange(n, dtype=np.int64)
        ep_indices = np.full(n, ep_idx, dtype=np.int64)
        task_indices = np.zeros(n, dtype=np.int64)

        if has_parquet:
            table = pa.table({
                "observation.state": pa.array([row.tolist() for row in state], type=pa.list_(pa.float32(), 6)),
                "action": pa.array([row.tolist() for row in action], type=pa.list_(pa.float32(), 6)),
                "timestamp": pa.array(timestamps),
                "frame_index": pa.array(frame_indices),
                "episode_index": pa.array(ep_indices),
                "index": pa.array(indices),
                "task_index": pa.array(task_indices),
            })
            pq.write_table(table, data_dir / f"episode_{ep_idx:06d}.parquet")
        else:
            with open(data_dir / f"episode_{ep_idx:06d}.csv", "w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                w.writerow(["index", "episode_index", "frame_index", "timestamp", "task_index"]
                           + [f"state_{j}" for j in JOINT_NAMES] + [f"action_{j}" for j in JOINT_NAMES])
                for i in range(n):
                    w.writerow([int(indices[i]), ep_idx, int(frame_indices[i]), round(float(timestamps[i]), 5), 0]
                               + [round(float(x), 6) for x in state[i]] + [round(float(x), 6) for x in action[i]])

        episodes_meta.append({"episode_index": ep_idx, "tasks": [task], "length": int(n)})
        total_frames += n

    info = {
        "codebase_version": "v2.1",
        "robot_type": "vulcan_sourccey_umi",
        "total_episodes": len(episodes_meta),
        "total_frames": int(total_frames),
        "total_tasks": 1,
        "total_videos": len(episodes_meta),
        "chunks_size": 1000,
        "fps": fps,
        "splits": {"train": f"0:{len(episodes_meta)}"},
        "data_path": "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}." + ("parquet" if has_parquet else "csv"),
        "video_path": "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4",
        "features": {
            "observation.state": {"dtype": "float32", "shape": [6], "names": JOINT_NAMES},
            "action": {"dtype": "float32", "shape": [6], "names": JOINT_NAMES},
            "observation.images.wrist": {
                "dtype": "video",
                "shape": video_shape,
                "names": ["height", "width", "channel"],
                "info": {"video.fps": fps, "video.codec": "mp4v"},
            },
            "timestamp": {"dtype": "float32", "shape": [1]},
            "frame_index": {"dtype": "int64", "shape": [1]},
            "episode_index": {"dtype": "int64", "shape": [1]},
            "index": {"dtype": "int64", "shape": [1]},
            "task_index": {"dtype": "int64", "shape": [1]},
        },
    }
    (meta_dir / "info.json").write_text(json.dumps(info, indent=2), encoding="utf-8")
    (meta_dir / "tasks.jsonl").write_text(json.dumps({"task_index": 0, "task": task}) + "\n", encoding="utf-8")
    (meta_dir / "episodes.jsonl").write_text(
        "\n".join(json.dumps(e) for e in episodes_meta) + "\n", encoding="utf-8"
    )

    result = {
        "out_dir": str(out_dir),
        "total_episodes": len(episodes_meta),
        "total_frames": int(total_frames),
        "format": "parquet" if has_parquet else "csv",
    }

    if push_to_hub:
        if not repo_id:
            raise ValueError("--repo-id is required when --push-to-hub is set")
        from huggingface_hub import HfApi
        api = HfApi()
        api.create_repo(repo_id=repo_id, repo_type="dataset", exist_ok=True)
        api.upload_folder(folder_path=str(out_dir), repo_id=repo_id, repo_type="dataset")
        result["repo_id"] = repo_id
        result["visualizer_url"] = f"https://huggingface.co/spaces/lerobot/visualize_dataset?dataset={repo_id}"

    return result


def main():
    ap = argparse.ArgumentParser(description="Grabette & LeRobot bridge for Sourccey UMI")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p_g = sub.add_parser("to-grabette", help="Convert one sourccey-umi episode to Grabette episode format")
    p_g.add_argument("--ep", required=True)
    p_g.add_argument("--out", required=True)
    p_g.add_argument("--hand", choices=("left", "right"), default="left")

    p_l = sub.add_parser("to-lerobot", help="Export episode(s) to a LeRobot dataset (+ optional HF Hub push)")
    p_l.add_argument("--eps", nargs="+", required=True)
    p_l.add_argument("--out", required=True)
    p_l.add_argument("--task", default="manipulate object")
    p_l.add_argument("--fps", type=float, default=30.0)
    p_l.add_argument("--repo-id", default=None)
    p_l.add_argument("--push-to-hub", action="store_true")

    args = ap.parse_args()
    if args.cmd == "to-grabette":
        print(json.dumps(export_grabette_episode(args.ep, args.out, args.hand), indent=2))
    else:
        print(json.dumps(export_lerobot_dataset(args.eps, args.out, args.task, args.fps, args.repo_id, args.push_to_hub), indent=2))


if __name__ == "__main__":
    main()
