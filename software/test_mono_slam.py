"""Tests for software/mono_slam.py (monocular 6-DoF pose recovery -> pose.csv -> LeRobot pipeline)."""
import csv
import json
import tempfile
from pathlib import Path

import cv2
import numpy as np

from mono_slam import estimate_monocular_pose, save_pose_csv


def _make_textured_aruco_video(ep_dir: Path, n_frames=16, w=320, h=240):
    ep_dir.mkdir(parents=True, exist_ok=True)
    t0 = 1_000_000_000
    (ep_dir / "meta.json").write_text(json.dumps({
        "t_start_monotonic_ns": t0,
        "width": w,
        "height": h,
        "n_frames": n_frames,
    }), encoding="utf-8")

    # Generate tabletop ArUco marker ID 10
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker = cv2.aruco.generateImageMarker(aruco_dict, 10, 96)
    marker_bgr = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)

    wr = cv2.VideoWriter(str(ep_dir / "video.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), 30.0, (w, h))
    for i in range(n_frames):
        frame = np.full((h, w, 3), 235, dtype=np.uint8)
        # Shift marker horizontally to simulate camera pan
        x0 = 90 + i * 3
        y0 = 70
        frame[y0:y0 + 96, x0:x0 + 96] = marker_bgr
        wr.write(frame)
    wr.release()

    with open(ep_dir / "frames.csv", "w", newline="", encoding="utf-8") as f:
        wcsv = csv.writer(f)
        wcsv.writerow(["frame", "t_monotonic_ns"])
        for i in range(n_frames):
            wcsv.writerow([i, t0 + int(i * 1e9 / 30.0)])


def test_mono_slam():
    with tempfile.TemporaryDirectory() as tmp:
        ep = Path(tmp) / "ep001"
        _make_textured_aruco_video(ep)
        t, pos, quat, lost, summary = estimate_monocular_pose(
            ep / "video.mp4", ep_dir=ep, table_marker_id=10, marker_size_m=0.08
        )
        assert summary["frames"] == 16
        assert summary["anchor_frames"] >= 14, f"expected ArUco anchors, got {summary}"
        assert summary["metric_anchored"] is True
        # Camera should move monotonically in X as the marker shifts across the image
        dx = pos[-1, 0] - pos[0, 0]
        assert abs(dx) > 0.01, f"expected measurable lateral motion, got dx={dx}"

        pose_csv = ep / "pose.csv"
        save_pose_csv(pose_csv, t, pos, quat, lost)
        assert pose_csv.exists() and len(pose_csv.read_text().splitlines()) == 17

    print("MONO SLAM OK:", summary)


if __name__ == "__main__":
    test_mono_slam()
