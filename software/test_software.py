import numpy as np, tempfile, os, sys
sys.path.insert(0, os.path.dirname(__file__))
import sts_angle as S
from retarget_ik import SourcceyArm

# ---- 1) reader/calibration with a fake servo (rising and falling direction, and crossing 4095->0)
class FakePH:  # mimics scservo_sdk.PacketHandler
    def __init__(self, seq): self.seq = list(seq); self.i = 0
    def write1ByteTxRx(self, *a): return 0, 0
    def read2ByteTxRx(self, port, sid, addr):
        v = self.seq[min(self.i, len(self.seq) - 1)]; self.i += 1; return v, 0, 0
S.CAL_FILE = __import__("pathlib").Path(tempfile.mkdtemp()) / "cal.json"
for closed, openr, name in ((100, 100 + int(65 * 4096 / 360), "raw rises"), (200, 200 - int(65 * 4096 / 360), "raw falls"), (4050, (4050 + int(65 * 4096 / 360)) % 4096, "wraps 4095->0")):
    s = S.StsAngle(handler=FakePH([0]), port_handler=object())
    cal = s.calibrate(closed, openr)
    qa, qb = s.raw_to_q(closed), s.raw_to_q(openr)
    assert abs(qa + 5) < 0.2 and abs(qb - 60) < 0.2, (name, qa, qb)
    print(f"ok reader [{name}]: closed={qa:.2f} deg, open={qb:.2f} deg, sign={cal['sign']:+.0f}")
print("jaw gap at 55 mm from the pivot: q=-5 ->", round(S.width_mm(-5, 55), 1), "mm | q=30 ->", round(S.width_mm(30, 55), 1), "mm | q=55 ->", round(S.width_mm(55, 55), 1), "mm")

# ---- 2) IK on the real Sourccey URDF: round trip (FK -> IK) with a synthetic trajectory
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools")); import common
urdf = os.environ.get("ARM_URDF") or common.default_urdf()
arm = SourcceyArm(urdf); rng = np.random.default_rng(0)
q_true = np.cumsum(rng.normal(0, 0.03, (60, 5)), 0) + np.array([0, 0.6, 0.9, -0.4, 0.3])
poses = [arm.fk(q) for q in q_true]
Q, EP, ER = arm.retarget(poses, np.eye(4), np.eye(4), q0=q_true[0] + 0.2)
print(f"synthetic IK (reachable): max position error {EP.max():.4f} mm, orientation {ER.max():.4f} deg, joints recovered within {np.abs(Q-q_true).max():.4f} rad")
assert EP.max() < 0.05 and ER.max() < 0.05
# unreachable pose: yaw 40 degrees out of the arm plane -> must REPORT the error, not hide it
from scipy.spatial.transform import Rotation as Rot
T = poses[0].copy(); R = np.eye(4); R[:3, :3] = Rot.from_euler("z", 40, degrees=True).as_matrix(); T = R @ T
q, ep, er = arm.ik(T, q_true[0]); print(f"unreachable 6D pose (yaw 40 deg): position error {ep:.1f} mm, orientation {er:.1f} deg  <- limit of the 5-joint arm")
print("ALL OK")
