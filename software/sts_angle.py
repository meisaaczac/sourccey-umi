"""UMI Sourccey angle reader: reads the STS3215 servo (torque OFF) and converts it to the Sourccey `gripper` joint.

Sourccey convention (URDF): q = 0 at the CAD zero, closed = -5 deg (touching the fixed jaw), fully open = +60 deg.
Usage:
  python sts_angle.py calibrate --port /dev/ttyACM0 --id 6      # follow the prompts
  python sts_angle.py log --port /dev/ttyACM0 --id 6 --out ep.csv --hz 200
Requires: pip install feetech-servo-sdk
"""
import argparse, csv, json, math, os, sys, time
from pathlib import Path

ADDR_TORQUE_ENABLE = 40       # STS3215
ADDR_PRESENT_POSITION = 56    # 2 bytes, 0..4095
COUNTS = 4096
Q_CLOSED_DEG, Q_OPEN_DEG = -5.0, 60.0
CAL_FILE = Path.home() / ".umi_sourccey" / "gripper_calibration.json"


def raw_to_deg(raw): return raw * 360.0 / COUNTS

def wrap180(d): return (d + 180.0) % 360.0 - 180.0


class StsAngle:
    def __init__(self, port="/dev/ttyACM0", servo_id=6, baud=1_000_000, handler=None, port_handler=None):
        if handler is None:
            from scservo_sdk import PortHandler, PacketHandler
            port_handler = PortHandler(port)
            if not port_handler.openPort(): raise RuntimeError(f"could not open {port}")
            port_handler.setBaudRate(baud)
            handler = PacketHandler(0)
        self.ph, self.pk, self.id = port_handler, handler, servo_id
        self.cal = self.load_cal()

    # --- low level
    def torque_off(self):
        self.pk.write1ByteTxRx(self.ph, self.id, ADDR_TORQUE_ENABLE, 0)

    def read_raw(self):
        val, res, err = self.pk.read2ByteTxRx(self.ph, self.id, ADDR_PRESENT_POSITION)
        if res != 0 or err != 0: raise IOError(f"read failed res={res} err={err}")
        return val & 0x0FFF

    def read_raw_avg(self, n=30, dt=0.01):
        s = c = 0.0
        for _ in range(n):
            a = math.radians(raw_to_deg(self.read_raw())); s += math.sin(a); c += math.cos(a); time.sleep(dt)
        return (math.degrees(math.atan2(s / n, c / n)) % 360.0) * COUNTS / 360.0   # circular mean, in counts

    # --- calibration: closed (-5) and open (stop, ~60) only to find the direction; the scale comes from physics (360/4096)
    def calibrate(self, closed_raw, open_raw):
        d = wrap180(raw_to_deg(open_raw) - raw_to_deg(closed_raw))
        sign = 1.0 if d > 0 else -1.0
        span = abs(d)
        cal = {"closed_raw": closed_raw, "sign": sign, "span_deg_measured": round(span, 2),
               "span_deg_expected": Q_OPEN_DEG - Q_CLOSED_DEG}
        if abs(span - (Q_OPEN_DEG - Q_CLOSED_DEG)) > 6:
            print(f"WARNING: measured travel {span:.1f} deg, expected 65. Check the horn/assembly before recording.", file=sys.stderr)
        CAL_FILE.parent.mkdir(parents=True, exist_ok=True)
        CAL_FILE.write_text(json.dumps(cal, indent=2)); self.cal = cal; return cal

    @staticmethod
    def load_cal():
        try: return json.loads(CAL_FILE.read_text())
        except Exception: return None

    def raw_to_q(self, raw, cal=None):
        cal = cal or self.cal
        if cal is None: raise RuntimeError("no calibration: run `calibrate`")
        d = wrap180(raw_to_deg(raw) - raw_to_deg(cal["closed_raw"]))
        return Q_CLOSED_DEG + cal["sign"] * d

    def read_q(self): return self.raw_to_q(self.read_raw())


def width_mm(q_deg, depth_mm=55, table=Path(__file__).with_name("sourccey_gripper_joint_vs_width.csv")):
    """Gap between the jaws (mm) at `depth_mm` from the pivot (40, 55 or 70): linear interpolation of the real-CAD table.
    Returns nan outside the table (the jaws are not parallel, so the gap depends on grasp depth)."""
    col = {40: 1, 55: 2, 70: 3}[depth_mm]
    pts = []
    for r in list(csv.reader(open(table)))[1:]:
        try: pts.append((float(r[0]), float(r[col])))
        except ValueError: pass
    pts.sort()
    if not pts or q_deg < pts[0][0] or q_deg > pts[-1][0]: return float("nan")
    for (a, wa), (b, wb) in zip(pts, pts[1:]):
        if a <= q_deg <= b: return wa + (wb - wa) * (q_deg - a) / (b - a)


def main():
    ap = argparse.ArgumentParser(); sub = ap.add_subparsers(dest="cmd", required=True)
    for n in ("calibrate", "log"):
        p = sub.add_parser(n); p.add_argument("--port", default="/dev/ttyACM0"); p.add_argument("--id", type=int, default=6)
    sub.choices["log"].add_argument("--out", required=True); sub.choices["log"].add_argument("--hz", type=float, default=200)
    a = ap.parse_args(); s = StsAngle(a.port, a.id); s.torque_off()
    if a.cmd == "calibrate":
        input("Squeeze the trigger until the gripper CLOSES on the fixed jaw and hold it. Press Enter... "); c = s.read_raw_avg()
        input("Release the trigger (the gripper opens to the stop). Press Enter... "); o = s.read_raw_avg()
        print(json.dumps(s.calibrate(c, o), indent=2))
    else:
        if s.cal is None: sys.exit("run `calibrate` first")
        dt = 1.0 / a.hz; nxt = time.monotonic()
        with open(a.out, "w", newline="") as f:
            w = csv.writer(f); w.writerow(["t_monotonic_ns", "t_wall_ns", "raw", "q_deg"])
            print("recording, Ctrl+C to stop")
            try:
                while True:
                    t0 = time.monotonic_ns(); raw = s.read_raw(); t1 = time.monotonic_ns()
                    w.writerow([(t0 + t1) // 2, time.time_ns(), raw, round(s.raw_to_q(raw), 3)])   # timestamp at the middle of the read
                    nxt += dt; time.sleep(max(0.0, nxt - time.monotonic()))
            except KeyboardInterrupt: pass

if __name__ == "__main__": main()
