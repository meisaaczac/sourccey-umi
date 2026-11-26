"""Merge the servo angle with the tracker pose, align in time, resample, convert to Sourccey joints (IK) and flag quality.
  python build_episode.py --ep ep001 --pose pose.csv --cfg cfg.json --out ep001/episode.npz
pose.csv: t (s, episode clock), x,y,z,qx,qy,qz,qw [,lost].  For UMI SLAM output use load_umi_slam().
cfg.json: see CFG_EXAMPLE below (lags come from sync_offset.py; T_tracker_tool comes from the CAD + your calibration)."""
import argparse, csv, json, os
import numpy as np
from scipy.spatial.transform import Rotation as Rot, Slerp
from retarget_ik import SourcceyArm

CFG_EXAMPLE = dict(urdf="sourccey-hardware/URDF/Arms/ArmLeft/ArmLeft.urdf", fps=30.0,
    angle_lag_s=0.0, pose_lag_s=0.0,
    T_tracker_tool=dict(xyz=[-0.051, -0.007, 0.034], quat_xyzw=[0, 0, 0, 1]),     # CAD mounting point; check the orientation of YOUR tracker
    T_base_world=dict(xyz=[0, 0, 0], quat_xyzw=[0, 0, 0, 1]),
    qc=dict(err_pos_mm_max=15.0, err_ori_deg_max=25.0, v_max_m_s=2.0, q_min=-5.5, q_max=60.5))

def T_from(d):
    T = np.eye(4); T[:3, :3] = Rot.from_quat(d["quat_xyzw"]).as_matrix(); T[:3, 3] = d["xyz"]; return T

def load_pose_csv(path):
    a = np.genfromtxt(path, delimiter=",", names=True); n = a.dtype.names
    lost = a["lost"].astype(bool) if "lost" in n else np.zeros(len(a), bool)
    return a["t"], np.c_[a["x"], a["y"], a["z"]], np.c_[a["qx"], a["qy"], a["qz"], a["qw"]], lost

def load_umi_slam(path, video_t0_s=0.0):
    """camera_trajectory.csv from the UMI pipeline (ORB-SLAM3). Columns by name; `timestamp` in video seconds."""
    a = np.genfromtxt(path, delimiter=",", names=True); n = a.dtype.names
    lost = a["is_lost"].astype(bool) if "is_lost" in n else np.zeros(len(a), bool)
    return a["timestamp"] + video_t0_s, np.c_[a["x"], a["y"], a["z"]], np.c_[a["q_x"], a["q_y"], a["q_z"], a["q_w"]], lost

def load_angle(ep):
    t0 = json.load(open(f"{ep}/meta.json"))["t_start_monotonic_ns"]
    a = np.genfromtxt(f"{ep}/angle.csv", delimiter=",", names=True)
    return (a["t_monotonic_ns"] - t0) / 1e9, a["q_deg"]

def build_episode(ep, pose, cfg, urdf=None):
    qc = cfg["qc"]; fps = cfg["fps"]
    ta, qa = load_angle(ep); ta = ta - cfg["angle_lag_s"]
    tp, pp, rp, lost = pose; tp = tp - cfg["pose_lag_s"]
    t0, t1 = max(ta[0], tp[0]), min(ta[-1], tp[-1]); t = np.arange(t0, t1, 1 / fps)
    if len(t) < 3: raise ValueError("no overlap between angle and pose")
    P = np.stack([np.interp(t, tp, pp[:, i]) for i in range(3)], 1)
    R = Slerp(tp, Rot.from_quat(rp))(t).as_matrix()
    idx = np.clip(np.searchsorted(tp, t), 1, len(tp) - 1)
    lost_g = lost[idx] | lost[idx - 1]                              # lost on either side of the interval
    gq = np.interp(t, ta, qa)
    arm = SourcceyArm(urdf or cfg["urdf"]); Tb, Tt = T_from(cfg["T_base_world"]), T_from(cfg["T_tracker_tool"])
    poses = []
    for i in range(len(t)):
        T = np.eye(4); T[:3, :3] = R[i]; T[:3, 3] = P[i]; poses.append(T)
    Q, EP, ER = arm.retarget(poses, Tb, Tt)
    tool = np.stack([Tb @ T @ Tt for T in poses])
    v = np.r_[0, np.linalg.norm(np.diff(tool[:, :3, 3], axis=0), axis=1) * fps]
    valid = (~lost_g) & (EP < qc["err_pos_mm_max"]) & (ER < qc["err_ori_deg_max"]) & (v < qc["v_max_m_s"]) & (gq > qc["q_min"]) & (gq < qc["q_max"])
    return dict(t=t, arm_q=Q, gripper_q_deg=gq, tool_pose=tool, err_pos_mm=EP, err_ori_deg=ER, speed=v, valid=valid,
                lost=lost_g), dict(frames=len(t), valid_pct=round(100 * valid.mean(), 1), err_pos_p95_mm=round(float(np.percentile(EP, 95)), 2),
                err_ori_p95_deg=round(float(np.percentile(ER, 95)), 2), q_range_deg=[round(float(gq.min()), 1), round(float(gq.max()), 1)])

if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--ep", required=True); ap.add_argument("--pose", required=True)
    ap.add_argument("--cfg", required=True); ap.add_argument("--out", required=True); a = ap.parse_args()
    cfg = json.load(open(a.cfg)); d, rep = build_episode(a.ep, load_pose_csv(a.pose), cfg)
    np.savez(a.out, **d); json.dump(rep, open(a.out + ".qc.json", "w"), indent=2); print(rep)
