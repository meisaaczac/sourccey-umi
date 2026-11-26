"""End-to-end test WITHOUT hardware: recorder with fake camera/servo -> video sync -> episode with IK."""
import os, sys, json, time, tempfile, threading, numpy as np, cv2
sys.path.insert(0, os.path.dirname(__file__))
import sts_angle as S, record_episode as RE, sync_offset as SO, build_episode as BE
from retarget_ik import SourcceyArm
from scipy.spatial.transform import Rotation as Rot
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools")); import common
ARM_URDF = os.environ.get("ARM_URDF") or common.default_urdf()
S.CAL_FILE = __import__("pathlib").Path(tempfile.mkdtemp()) / "cal.json"

def q_true(t): return 27.5 + 30 * np.sin(2 * np.pi * 0.45 * t) + 8 * np.sin(2 * np.pi * 1.3 * t + 1)     # real open/close motion, -5..60 deg
LAT = 0.15                                                                                              # simulated camera delay
class FakePH:
    t0 = time.monotonic()
    def write1ByteTxRx(self, *a): return 0, 0
    def read2ByteTxRx(self, port, sid, addr):
        q = q_true(time.monotonic() - self.t0); raw = int(round(((q + 5) / 360 * 4096 + 1000))) % 4096; return raw, 0, 0
servo = S.StsAngle(handler=FakePH(), port_handler=object()); servo.calibrate(int(round(1000)), int(round(1000 + 65 / 360 * 4096)))

class FakeCap:
    def __init__(self, t0): self.t0 = t0; self.n = 0
    def read(self):
        time.sleep(1 / 30); t = time.monotonic() - self.t0 - LAT; q = q_true(t)
        fr = np.full((480, 640, 3), 120, np.uint8); d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        for mid, x in ((1, 120), (0, int(250 + 3.0 * q))):
            m = cv2.cvtColor(cv2.aruco.generateImageMarker(d, mid, 100), cv2.COLOR_GRAY2BGR); m = cv2.copyMakeBorder(m, 20, 20, 20, 20, cv2.BORDER_CONSTANT, value=(255, 255, 255))
            fr[200:340, x:x + 140] = m
        return True, fr

# ---------- 1) recorder + sync
ep = tempfile.mkdtemp(); FakePH.t0 = time.monotonic()
wr = cv2.VideoWriter(f"{ep}/video.avi", cv2.VideoWriter_fourcc(*"MJPG"), 30, (640, 480))
info = RE.record(ep, FakeCap(FakePH.t0), wr, servo, seconds=4.0, hz=200, meta=dict(cal=servo.cal))
print("recorded:", {k: (round(v, 1) if isinstance(v, float) else v) for k, v in info.items() if k in ("n_angle", "n_frames", "angle_hz_measured", "fps_measured", "angle_read_errors")})
assert info["n_frames"] > 90 and info["angle_hz_measured"] > 100
fr = np.genfromtxt(f"{ep}/frames.csv", delimiter=",", names=True); t0 = info["t_start_monotonic_ns"]
tv, dv = SO.video_opening_signal(f"{ep}/video.avi", frame_times=(fr["t_monotonic_ns"][:] - t0) / 1e9)
ta, qa = BE.load_angle(ep)
lag, score = SO.estimate_lag(ta, qa, tv, dv, max_lag=0.6)
print(f"estimated video delay: {lag*1000:.0f} ms (true {LAT*1000:.0f} ms), correlation {score:.3f}")
assert abs(lag - LAT) < 0.03 and score > 0.9

# ---------- 2) episode: synthetic pose from known joints, with a clock delay
arm = SourcceyArm(ARM_URDF); fps = 30.0; tt = np.arange(0.3, 3.7, 1 / fps)
qj = np.array([0.0, 0.6, 0.9, -0.4, 0.3]) + 0.35 * np.c_[np.sin(tt), np.sin(0.7 * tt), np.sin(0.5 * tt + 1), np.sin(0.9 * tt), np.sin(1.3 * tt)]
cfg = json.loads(json.dumps(BE.CFG_EXAMPLE)); cfg["angle_lag_s"] = 0.0; cfg["pose_lag_s"] = 0.05; cfg["urdf"] = ARM_URDF
cfg["T_base_world"] = dict(xyz=[0.4, -0.2, 0.1], quat_xyzw=Rot.from_euler("z", 20, degrees=True).as_quat().tolist())
cfg["T_tracker_tool"] = dict(xyz=[-0.051, -0.007, 0.034], quat_xyzw=Rot.from_euler("xyz", [5, -3, 10], degrees=True).as_quat().tolist())
Tb, Tt = BE.T_from(cfg["T_base_world"]), BE.T_from(cfg["T_tracker_tool"]); rows = []
for t, q in zip(tt, qj):
    Tw = np.linalg.inv(Tb) @ arm.fk(q) @ np.linalg.inv(Tt); qu = Rot.from_matrix(Tw[:3, :3]).as_quat()
    rows.append([t + 0.05, *Tw[:3, 3], *qu, 0])                       # tracker clock ahead by 50 ms
np.savetxt(f"{ep}/pose.csv", np.array(rows), delimiter=",", header="t,x,y,z,qx,qy,qz,qw,lost", comments="")
d, rep = BE.build_episode(ep, BE.load_pose_csv(f"{ep}/pose.csv"), cfg)
print("episode:", rep)
qtrue = np.stack([np.interp(d["t"], tt, qj[:, i]) for i in range(5)], 1)
print(f"recovered joints vs truth: max error {np.abs(d['arm_q']-qtrue).max():.4f} rad; valid {rep['valid_pct']}%")
assert np.abs(d["arm_q"] - qtrue).max() < 0.01 and rep["valid_pct"] > 90
print("PIPELINE OK")
