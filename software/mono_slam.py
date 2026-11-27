"""Monocular 6-DoF pose estimation for Sourccey UMI's single 20x20 mm RGB camera.

Replaces Grabette's RGB-D RTAB-Map requirement when using ONLY the Sourccey
20x20 mm wrist camera (no GoPro, no depth camera):
  1. Tabletop ArUco Metric Anchor (preferred, $0): place a printed ArUco marker
     (e.g. ID 10, 80 mm) flat on the workspace table. `cv2.solvePnP` recovers
     metric 6-DoF camera pose whenever the marker is visible.
  2. ORB Visual Odometry (`cv2.findEssentialMat` + `cv2.recoverPose`): bridges
     frames where the tabletop marker is out of view, scaled cleanly to the
     latest metric anchor (or a configurable step scale if no marker is used).

Outputs `pose.csv` (`t,x,y,z,qx,qy,qz,qw,lost`) directly compatible with
`software/build_episode.py` and `space/app.py`.

Usage:
  python software/mono_slam.py --ep ep001 --out ep001/pose.csv --table-marker-id 10 --marker-mm 80
"""
import argparse
import csv
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as Rot


def default_camera_matrix(w: int, h: int, hfov_deg: float = 78.0) -> np.ndarray:
    """Pinhole intrinsics approximation for a 20x20 mm wide-angle Sourccey USB camera."""
    fx = (w * 0.5) / np.tan(np.radians(hfov_deg * 0.5))
    fy = fx
    return np.array([[fx, 0.0, w * 0.5], [0.0, fy, h * 0.5], [0.0, 0.0, 1.0]], dtype=np.float64)


def _load_frame_times(ep_dir: Path, n_frames: int, fps: float) -> np.ndarray:
    f_csv = ep_dir / "frames.csv"
    meta_p = ep_dir / "meta.json"
    if f_csv.exists() and meta_p.exists():
        t0_ns = int(json.loads(meta_p.read_text(encoding="utf-8"))["t_start_monotonic_ns"])
        ts = []
        with open(f_csv, newline="", encoding="utf-8") as f:
            for row in csv.DictReader(f):
                ts.append((int(row["t_monotonic_ns"]) - t0_ns) / 1e9)
        if len(ts) >= n_frames:
            return np.asarray(ts[:n_frames], dtype=np.float64)
    return np.arange(n_frames, dtype=np.float64) / max(fps, 1.0)


def estimate_monocular_pose(
    video_path: str | Path,
    ep_dir: str | Path | None = None,
    table_marker_id: int = 10,
    marker_size_m: float = 0.08,
    hfov_deg: float = 78.0,
    vo_step_scale_m: float = 0.006,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, dict]:
    """Estimate 6-DoF camera trajectory from a single RGB wrist video.

    Returns:
      t[N], pos[N,3], quat_xyzw[N,4], lost[N], summary_dict
    """
    video_path = Path(video_path)
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise RuntimeError(f"Could not open video: {video_path}")

    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    K = default_camera_matrix(w, h, hfov_deg=hfov_deg)
    dist = np.zeros(5, dtype=np.float64)

    # ArUco detector for tabletop metric anchor
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())
    half = marker_size_m * 0.5
    obj_pts = np.array(
        [[-half, half, 0.0], [half, half, 0.0], [half, -half, 0.0], [-half, -half, 0.0]],
        dtype=np.float64,
    )

    # ORB + BFMatcher for visual odometry between marker sightings
    orb = cv2.ORB_create(nfeatures=800)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    T_cur = np.eye(4, dtype=np.float64)
    prev_kp, prev_des = None, None
    poses, lost_flags = [], []
    n_anchor, n_vo, n_lost = 0, 0, 0

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 1. Try metric tabletop ArUco anchor first
        got_anchor = False
        corners, ids, _ = detector.detectMarkers(gray)
        if ids is not None:
            for c_arr, mid in zip(corners, ids.ravel()):
                if int(mid) == table_marker_id:
                    ok_pnp, rvec, tvec = cv2.solvePnP(
                        obj_pts, c_arr.reshape(4, 2).astype(np.float64), K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE
                    )
                    if ok_pnp:
                        R_cm, _ = cv2.Rodrigues(rvec)
                        # Invert marker-in-camera -> camera-in-table (world)
                        R_wc = R_cm.T
                        t_wc = -R_cm.T @ tvec.reshape(3)
                        T_cur = np.eye(4, dtype=np.float64)
                        T_cur[:3, :3] = R_wc
                        T_cur[:3, 3] = t_wc
                        got_anchor = True
                        n_anchor += 1
                        break

        # 2. Fallback to ORB visual odometry if anchor not visible
        kp, des = orb.detectAndCompute(gray, None)
        is_lost = False
        if not got_anchor:
            if prev_des is not None and des is not None and len(prev_kp) >= 12 and len(kp) >= 12:
                matches = sorted(bf.match(prev_des, des), key=lambda m: m.distance)[:120]
                if len(matches) >= 12:
                    pts1 = np.float32([prev_kp[m.queryIdx].pt for m in matches])
                    pts2 = np.float32([kp[m.trainIdx].pt for m in matches])
                    E, mask = cv2.findEssentialMat(pts1, pts2, K, method=cv2.RANSAC, prob=0.99, threshold=1.5)
                    if E is not None and E.shape == (3, 3):
                        inliers, R_rel, t_rel, _ = cv2.recoverPose(E, pts1, pts2, K, mask=mask)
                        if inliers >= 8:
                            T_step = np.eye(4, dtype=np.float64)
                            T_step[:3, :3] = R_rel.T
                            T_step[:3, 3] = (-R_rel.T @ t_rel.reshape(3)) * vo_step_scale_m
                            T_cur = T_cur @ T_step
                            n_vo += 1
                        else:
                            is_lost = True
                    else:
                        is_lost = True
                else:
                    is_lost = True
            elif len(poses) > 0:
                is_lost = True
            if is_lost:
                n_lost += 1

        prev_kp, prev_des = kp, des
        poses.append(T_cur.copy())
        lost_flags.append(is_lost)

    cap.release()
    n = len(poses)
    if n == 0:
        raise RuntimeError("Video contained 0 readable frames")

    t = _load_frame_times(Path(ep_dir) if ep_dir else video_path.parent, n, fps)
    pos = np.array([T[:3, 3] for T in poses], dtype=np.float64)
    quat = np.array([Rot.from_matrix(T[:3, :3]).as_quat() for T in poses], dtype=np.float64)
    lost = np.asarray(lost_flags, dtype=bool)

    summary = {
        "frames": int(n),
        "anchor_frames": int(n_anchor),
        "vo_frames": int(n_vo),
        "lost_frames": int(n_lost),
        "metric_anchored": bool(n_anchor > 0),
        "mode": "aruco_metric_hybrid" if n_anchor > 0 else "monocular_vo",
    }
    return t, pos, quat, lost, summary


def save_pose_csv(out_path: str | Path, t: np.ndarray, pos: np.ndarray, quat: np.ndarray, lost: np.ndarray) -> None:
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t", "x", "y", "z", "qx", "qy", "qz", "qw", "lost"])
        for ti, p, q, l in zip(t, pos, quat, lost):
            w.writerow([
                round(float(ti), 6),
                round(float(p[0]), 6), round(float(p[1]), 6), round(float(p[2]), 6),
                round(float(q[0]), 6), round(float(q[1]), 6), round(float(q[2]), 6), round(float(q[3]), 6),
                int(bool(l)),
            ])


def main():
    ap = argparse.ArgumentParser(description="Monocular 6-DoF pose estimator for Sourccey UMI (20x20 mm RGB camera)")
    ap.add_argument("--ep", required=True, help="Episode directory containing video.mp4")
    ap.add_argument("--out", default=None, help="Output pose.csv path (default: <ep>/pose.csv)")
    ap.add_argument("--table-marker-id", type=int, default=10, help="ArUco 4x4_50 ID on table for metric scale")
    ap.add_argument("--marker-mm", type=float, default=80.0, help="Printed tabletop ArUco marker side length (mm)")
    ap.add_argument("--hfov", type=float, default=78.0, help="Camera horizontal FOV in degrees")
    args = ap.parse_args()

    ep_dir = Path(args.ep)
    out_path = Path(args.out) if args.out else ep_dir / "pose.csv"
    t, pos, quat, lost, summary = estimate_monocular_pose(
        ep_dir / "video.mp4",
        ep_dir=ep_dir,
        table_marker_id=args.table_marker_id,
        marker_size_m=args.marker_mm / 1000.0,
        hfov_deg=args.hfov,
    )
    save_pose_csv(out_path, t, pos, quat, lost)
    print(json.dumps({"pose_csv": str(out_path), **summary}, indent=2))


if __name__ == "__main__":
    main()
