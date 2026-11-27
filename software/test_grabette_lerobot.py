"""Tests for software/grabette_lerobot.py (Grabette angle adapter, Grabette exporter, LeRobot exporter)."""
import csv
import json
import math
import tempfile
from pathlib import Path

import cv2
import numpy as np

from grabette_lerobot import StsAngleCapture, export_grabette_episode, export_lerobot_dataset


class _FakeSts:
    def __init__(self):
        self.vals = [60.0, 30.0, -5.0]
        self.i = 0

    def read_q(self):
        v = self.vals[min(self.i, len(self.vals) - 1)]
        self.i += 1
        return v


def _make_fake_episode(ep_dir: Path, n_frames=12):
    ep_dir.mkdir(parents=True, exist_ok=True)
    t0 = 1_000_000_000
    (ep_dir / "meta.json").write_text(json.dumps({
        "t_start_monotonic_ns": t0,
        "width": 160,
        "height": 120,
        "n_frames": n_frames,
    }), encoding="utf-8")

    # Fake wrist video
    wr = cv2.VideoWriter(str(ep_dir / "video.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), 30.0, (160, 120))
    for i in range(n_frames):
        wr.write(np.full((120, 160, 3), i * 10, dtype=np.uint8))
    wr.release()

    with open(ep_dir / "frames.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["frame", "t_monotonic_ns"])
        for i in range(n_frames):
            w.writerow([i, t0 + int(i * 1e9 / 30.0)])

    with open(ep_dir / "angle.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t_monotonic_ns", "raw", "q_deg"])
        for i in range(n_frames * 3):
            q = 60.0 - 65.0 * (i / max(1, n_frames * 3 - 1))
            w.writerow([t0 + int(i * 1e9 / 90.0), 2000 + i, round(q, 3)])


def test_all():
    # 1. Angle convention mapping: 60 deg open -> 0 rad; -5 deg closed -> +65 deg in rad
    assert abs(StsAngleCapture.q_deg_to_grabette_rad(60.0)) < 1e-9
    assert abs(StsAngleCapture.q_deg_to_grabette_rad(-5.0) - math.radians(65.0)) < 1e-9

    # 2. Live capture thread with fake STS3215
    cap = StsAngleCapture(_FakeSts(), sample_rate_hz=200.0)
    cap.start_capture()
    import time
    time.sleep(0.05)
    samples = cap.stop()
    assert len(samples) >= 3, f"too few samples: {len(samples)}"
    assert "cts" in samples[0] and len(samples[0]["value"]) == 2

    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        ep = tmp / "ep001"
        _make_fake_episode(ep)

        # 3. Grabette canonical episode export
        g_out = tmp / "grabette_ep001"
        g_meta = export_grabette_episode(ep, g_out)
        assert g_meta["frame_count"] == 12
        for req in ("raw_video.mp4", "frame_timestamps.json", "angle_data.json", "frames.json", "metadata.json"):
            assert (g_out / req).exists(), f"missing {req}"

        # 4. LeRobot dataset export
        lr_out = tmp / "lerobot_ds"
        res = export_lerobot_dataset([ep], lr_out, task="pick red block")
        assert res["total_episodes"] == 1
        assert res["total_frames"] == 12
        assert (lr_out / "meta" / "info.json").exists()
        assert (lr_out / "meta" / "episodes.jsonl").exists()
        assert (lr_out / "meta" / "tasks.jsonl").exists()
        assert (lr_out / "videos" / "chunk-000" / "observation.images.wrist" / "episode_000000.mp4").exists()

    print("GRABETTE + LEROBOT BRIDGE OK")


if __name__ == "__main__":
    test_all()
