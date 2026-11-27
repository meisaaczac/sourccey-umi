"""Bimanual (Dual-Arm) recorder for Sourccey UMI (LEFT + RIGHT simultaneously).

Reads two daisy-chained Feetech STS3215 servos on the SAME serial bus (e.g.
ID 6 for left gripper, ID 16 for right gripper) and two 20x20 mm USB wrist
cameras on one shared monotonic clock.

Writes:
  video_left.mp4, video_right.mp4
  frames_left.csv, frames_right.csv
  angle_bimanual.csv   (t_monotonic_ns, raw_left, q_left_deg, raw_right, q_right_deg)
  meta.json

Usage:
  python software/record_bimanual.py --out ep_bi001 --port /dev/ttyACM0 \
    --id-left 6 --id-right 16 --cam-left 0 --cam-right 2 --seconds 30
"""
import argparse
import csv
import json
import os
import threading
import time


class DualAngleThread(threading.Thread):
    """Polls left + right STS3215 servos sequentially on one shared bus."""

    def __init__(self, reader_left, reader_right, hz=150.0):
        super().__init__(daemon=True)
        self.rl = reader_left
        self.rr = reader_right
        self.dt = 1.0 / hz
        self.rows = []
        self.stop = threading.Event()
        self.errors = 0

    def run(self):
        nxt = time.monotonic()
        while not self.stop.is_set():
            t0 = time.monotonic_ns()
            try:
                raw_l = self.rl.read_raw()
                raw_r = self.rr.read_raw()
            except Exception:
                self.errors += 1
                raw_l = raw_r = None
            t1 = time.monotonic_ns()
            if raw_l is not None and raw_r is not None:
                self.rows.append((
                    (t0 + t1) // 2,
                    raw_l,
                    round(self.rl.raw_to_q(raw_l), 3),
                    raw_r,
                    round(self.rr.raw_to_q(raw_r), 3),
                ))
            nxt += self.dt
            time.sleep(max(0.0, nxt - time.monotonic()))


class _CamThread(threading.Thread):
    def __init__(self, cap, writer):
        super().__init__(daemon=True)
        self.cap = cap
        self.w = writer
        self.rows = []
        self.stop = threading.Event()
        self.drops = 0

    def run(self):
        i = 0
        while not self.stop.is_set():
            ok, frame = self.cap.read()
            t = time.monotonic_ns()
            if not ok:
                self.drops += 1
                continue
            self.w.write(frame)
            self.rows.append((i, t))
            i += 1


def record_bimanual(out, cap_l, wr_l, cap_r, wr_r, reader_l, reader_r, seconds, hz=150.0, meta=None, wait_enter=False):
    os.makedirs(out, exist_ok=True)
    ta = DualAngleThread(reader_l, reader_r, hz)
    cl = _CamThread(cap_l, wr_l)
    cr = _CamThread(cap_r, wr_r)

    t_start = time.monotonic_ns()
    ta.start()
    cl.start()
    cr.start()

    if wait_enter:
        input("Recording bimanual episode. Press Enter to stop... ")
    else:
        time.sleep(seconds)

    ta.stop.set()
    cl.stop.set()
    cr.stop.set()
    ta.join(2)
    cl.join(2)
    cr.join(2)
    wr_l.release()
    wr_r.release()

    with open(f"{out}/angle_bimanual.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t_monotonic_ns", "raw_left", "q_left_deg", "raw_right", "q_right_deg"])
        w.writerows(ta.rows)
    for tag, rows in (("left", cl.rows), ("right", cr.rows)):
        with open(f"{out}/frames_{tag}.csv", "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["frame", "t_monotonic_ns"])
            w.writerows(rows)

    info = dict(
        meta or {},
        bimanual=True,
        t_start_monotonic_ns=t_start,
        n_angle=len(ta.rows),
        n_frames_left=len(cl.rows),
        n_frames_right=len(cr.rows),
        angle_read_errors=ta.errors,
        camera_drops_left=cl.drops,
        camera_drops_right=cr.drops,
    )
    with open(f"{out}/meta.json", "w", encoding="utf-8") as f:
        json.dump(info, f, indent=2)
    return info


if __name__ == "__main__":
    import cv2
    import sts_angle

    ap = argparse.ArgumentParser(description="Bimanual Sourccey UMI recorder (LEFT + RIGHT on shared bus)")
    ap.add_argument("--out", required=True)
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--id-left", type=int, default=6)
    ap.add_argument("--id-right", type=int, default=16)
    ap.add_argument("--cam-left", type=int, default=0)
    ap.add_argument("--cam-right", type=int, default=2)
    ap.add_argument("--seconds", type=float, default=0)
    ap.add_argument("--hz", type=float, default=150.0)
    ap.add_argument("--fps", type=float, default=30.0)
    a = ap.parse_args()

    # Share one PortHandler + PacketHandler across both daisy-chained servos
    sl = sts_angle.StsAngle(a.port, a.id_left)
    sr = sts_angle.StsAngle(a.port, a.id_right, handler=sl.pk, port_handler=sl.ph)
    sl.torque_off()
    sr.torque_off()
    if sl.cal is None or sr.cal is None:
        raise SystemExit("Run `python software/sts_angle.py calibrate` first.")

    cap_l = cv2.VideoCapture(a.cam_left)
    cap_r = cv2.VideoCapture(a.cam_right)
    if not cap_l.isOpened() or not cap_r.isOpened():
        raise SystemExit("Could not open both left and right USB wrist cameras.")

    os.makedirs(a.out, exist_ok=True)
    wl, hl = int(cap_l.get(cv2.CAP_PROP_FRAME_WIDTH)) or 640, int(cap_l.get(cv2.CAP_PROP_FRAME_HEIGHT)) or 480
    wr_w, wr_h = int(cap_r.get(cv2.CAP_PROP_FRAME_WIDTH)) or 640, int(cap_r.get(cv2.CAP_PROP_FRAME_HEIGHT)) or 480
    vw_l = cv2.VideoWriter(f"{a.out}/video_left.mp4", cv2.VideoWriter_fourcc(*"mp4v"), a.fps, (wl, hl))
    vw_r = cv2.VideoWriter(f"{a.out}/video_right.mp4", cv2.VideoWriter_fourcc(*"mp4v"), a.fps, (wr_w, wr_h))

    res = record_bimanual(
        a.out, cap_l, vw_l, cap_r, vw_r, sl, sr, a.seconds, a.hz,
        meta={"width_left": wl, "height_left": hl, "width_right": wr_w, "height_right": wr_h},
        wait_enter=a.seconds <= 0,
    )
    print(json.dumps(res, indent=2))
