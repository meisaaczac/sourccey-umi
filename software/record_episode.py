"""Record ONE episode: Sourccey 20x20 mm wrist camera + STS3215 servo angle on the SAME monotonic clock.
  python record_episode.py --out ep001 --camera 0 --port /dev/ttyACM0 --id 6 --seconds 60
Writes: video.mp4, frames.csv (time per frame), angle.csv (t, raw, q), meta.json.
If you use a separate pose stream, align it afterwards with sync_offset.py."""
import argparse, csv, json, threading, time, os
import numpy as np

class AngleThread(threading.Thread):
    def __init__(self, reader, hz):
        super().__init__(daemon=True); self.r, self.dt, self.rows, self.stop, self.errors = reader, 1.0 / hz, [], threading.Event(), 0
    def run(self):
        nxt = time.monotonic()
        while not self.stop.is_set():
            t0 = time.monotonic_ns()
            try: raw = self.r.read_raw()
            except Exception: self.errors += 1; raw = None
            t1 = time.monotonic_ns()
            if raw is not None: self.rows.append(((t0 + t1) // 2, raw, self.r.raw_to_q(raw)))
            nxt += self.dt; time.sleep(max(0.0, nxt - time.monotonic()))

class CameraThread(threading.Thread):
    def __init__(self, cap, writer):
        super().__init__(daemon=True); self.cap, self.w, self.rows, self.stop, self.drops = cap, writer, [], threading.Event(), 0
    def run(self):
        i = 0
        while not self.stop.is_set():
            ok, frame = self.cap.read(); t = time.monotonic_ns()      # frame arrival time
            if not ok: self.drops += 1; continue
            self.w.write(frame); self.rows.append((i, t)); i += 1

def record(out, cap, writer, angle_reader, seconds, hz=200, meta=None, wait_enter=False):
    os.makedirs(out, exist_ok=True)
    a, c = AngleThread(angle_reader, hz), CameraThread(cap, writer)
    t_start = time.monotonic_ns(); a.start(); c.start()
    if wait_enter: input("Recording. Press Enter to stop... ")
    else: time.sleep(seconds)
    a.stop.set(); c.stop.set(); a.join(2); c.join(2); writer.release()
    with open(f"{out}/angle.csv", "w", newline="") as f:
        w = csv.writer(f); w.writerow(["t_monotonic_ns", "raw", "q_deg"]); w.writerows(a.rows)
    with open(f"{out}/frames.csv", "w", newline="") as f:
        w = csv.writer(f); w.writerow(["frame", "t_monotonic_ns"]); w.writerows(c.rows)
    info = dict(meta or {}, t_start_monotonic_ns=t_start, wall_start_ns=time.time_ns() - (time.monotonic_ns() - t_start),
                n_angle=len(a.rows), n_frames=len(c.rows), angle_read_errors=a.errors, camera_drops=c.drops,
                angle_hz_measured=len(a.rows) / max(1e-9, (a.rows[-1][0] - a.rows[0][0]) / 1e9) if len(a.rows) > 1 else 0,
                fps_measured=len(c.rows) / max(1e-9, (c.rows[-1][1] - c.rows[0][1]) / 1e9) if len(c.rows) > 1 else 0)
    json.dump(info, open(f"{out}/meta.json", "w"), indent=2); return info

if __name__ == "__main__":
    import cv2, sts_angle
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True); ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--port", default="/dev/ttyACM0"); ap.add_argument("--id", type=int, default=6)
    ap.add_argument("--seconds", type=float, default=0); ap.add_argument("--hz", type=float, default=200)
    ap.add_argument("--w", type=int, default=1280); ap.add_argument("--h", type=int, default=720); ap.add_argument("--fps", type=float, default=30)
    a = ap.parse_args()
    s = sts_angle.StsAngle(a.port, a.id); s.torque_off()
    if s.cal is None: raise SystemExit("run `python sts_angle.py calibrate` first")
    cap = cv2.VideoCapture(a.camera)
    if not cap.isOpened(): raise SystemExit(f"could not open camera {a.camera}")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, a.w); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, a.h); cap.set(cv2.CAP_PROP_FPS, a.fps)
    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or a.w
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or a.h
    os.makedirs(a.out, exist_ok=True)
    wr = cv2.VideoWriter(f"{a.out}/video.mp4", cv2.VideoWriter_fourcc(*"mp4v"), a.fps, (actual_w, actual_h))
    print(record(a.out, cap, wr, s, a.seconds, a.hz, dict(cal=s.cal, width=actual_w, height=actual_h), wait_enter=a.seconds <= 0))
