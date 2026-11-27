"""Tests for v2 features: bimanual recorder, latency matching, relative EE actions, QC gate, portable Pi node."""
import csv
import json
import math
import tempfile
import time
from pathlib import Path

import cv2
import numpy as np

from grabette_lerobot import check_episode_qc, compute_relative_ee_actions, export_lerobot_dataset
from portable_node import ButtonTrigger, FeedbackController, ImuLoggerThread, ImuReader, write_imu_csv
from record_bimanual import record_bimanual


class _FakeReader:
    def __init__(self, base=1500):
        self.base = base
        self.k = 0

    def read_raw(self):
        self.k += 1
        return (self.base + self.k * 2) & 0x0FFF

    def raw_to_q(self, raw):
        return -5.0 + (raw - self.base) * 360.0 / 4096.0


class _FakeCap:
    def read(self):
        time.sleep(0.015)
        return True, np.full((60, 80, 3), 120, dtype=np.uint8)


class _FakeWriter:
    def __init__(self):
        self.n = 0

    def write(self, fr):
        self.n += 1

    def release(self):
        pass


def test_bimanual():
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / "ep_bi"
        info = record_bimanual(
            str(out), _FakeCap(), _FakeWriter(), _FakeCap(), _FakeWriter(),
            _FakeReader(1000), _FakeReader(2000), seconds=0.12, hz=100.0
        )
        assert info["bimanual"] is True
        assert info["n_angle"] >= 5
        assert (out / "angle_bimanual.csv").exists()
        assert (out / "frames_left.csv").exists() and (out / "frames_right.csv").exists()


def test_latency_relative_and_qc():
    # Relative EE sanity: constant +1 cm/step along +x in tool frame
    n = 10
    poses = np.repeat(np.eye(4)[None, :, :], n, axis=0)
    for i in range(n):
        poses[i, 0, 3] = 0.01 * i
    gq = np.linspace(1.0, 0.0, n)
    rel = compute_relative_ee_actions(poses, gq, step_shift=2)
    assert abs(rel[0, 0] - 0.02) < 1e-6, f"expected +0.02 m shift, got {rel[0, 0]}"
    assert abs(rel[0, 6] - gq[2]) < 1e-6

    # QC gate test: 1 passing episode + 1 failing episode
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        for name, valid_pct, p95 in (("ep_good", 96.0, 4.2), ("ep_bad", 45.0, 28.5)):
            ep = tmp / name
            ep.mkdir()
            t0 = 1_000_000_000
            (ep / "meta.json").write_text(json.dumps({"t_start_monotonic_ns": t0, "width": 80, "height": 60}))
            with open(ep / "frames.csv", "w", newline="") as f:
                w = csv.writer(f); w.writerow(["frame", "t_monotonic_ns"])
                for i in range(n): w.writerow([i, t0 + int(i * 1e9 / 30.0)])
            with open(ep / "angle.csv", "w", newline="") as f:
                w = csv.writer(f); w.writerow(["t_monotonic_ns", "raw", "q_deg"])
                for i in range(n): w.writerow([t0 + int(i * 1e9 / 30.0), 1000, 30.0])
            np.savez(
                ep / "episode.npz",
                t=np.arange(n) / 30.0,
                arm_q=np.zeros((n, 5)),
                gripper_q_deg=np.full(n, 30.0),
                tool_pose=poses,
                valid=np.ones(n, dtype=bool),
                err_pos_mm=np.full(n, p95),
            )
            (ep / "episode.npz.qc.json").write_text(json.dumps({"valid_pct": valid_pct, "err_pos_p95_mm": p95}))

        ok_g, _ = check_episode_qc(tmp / "ep_good")
        ok_b, _ = check_episode_qc(tmp / "ep_bad")
        assert ok_g is True and ok_b is False

        res = export_lerobot_dataset(
            [tmp / "ep_good", tmp / "ep_bad"], tmp / "ds",
            action_latency_ms=66.7, enforce_qc=True
        )
        assert res["total_episodes"] == 1
        assert len(res["skipped_episodes"]) == 1 and res["skipped_episodes"][0]["episode"] == "ep_bad"
        assert res["action_step_shift"] == 3
        assert res["has_relative_ee"] is True


def test_portable_node():
    btn = ButtonTrigger(debounce_s=0.1, mock=True)
    assert btn.handle_press(1.0) == "start"
    assert btn.handle_press(1.02) is None  # debounced
    assert btn.handle_press(1.2) == "stop"

    fb = FeedbackController(mock=True)
    fb.set_state("recording")
    assert fb.state == "recording"

    imu = ImuReader("mock")
    th = ImuLoggerThread(imu, hz=100.0)
    th.start()
    time.sleep(0.06)
    th.stop.set()
    th.join(1)
    assert len(th.rows) >= 3
    with tempfile.TemporaryDirectory() as tmp:
        p = Path(tmp) / "imu.csv"
        write_imu_csv(p, th.rows)
        assert p.exists() and len(p.read_text().splitlines()) == len(th.rows) + 1


if __name__ == "__main__":
    test_bimanual()
    test_latency_relative_and_qc()
    test_portable_node()
    print("V2 FEATURES OK (bimanual, latency, relative EE, QC gate, portable Pi node)")
