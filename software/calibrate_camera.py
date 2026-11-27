"""Chessboard intrinsic camera calibration for the Sourccey 20x20 mm wrist camera.

Captures chessboard frames from the live USB camera (or a folder of images) and
writes `camera_calib.json` (K matrix + radial/tangential distortion coefficients)
used by `software/mono_slam.py --calib camera_calib.json`.

Usage:
  python software/calibrate_camera.py --camera 0 --cols 9 --rows 6 --square-mm 20 --out camera_calib.json
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def calibrate_from_frames(frames: list[np.ndarray], cols: int = 9, rows: int = 6, square_m: float = 0.020) -> dict:
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_m
    objpoints, imgpoints = [], []
    img_shape = None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    for fr in frames:
        gray = cv2.cvtColor(fr, cv2.COLOR_BGR2GRAY) if fr.ndim == 3 else fr
        img_shape = gray.shape[::-1]
        ok, corners = cv2.findChessboardCorners(gray, (cols, rows), None)
        if ok:
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            objpoints.append(objp)
            imgpoints.append(corners2)

    if len(objpoints) < 5:
        raise RuntimeError(f"Need at least 5 valid chessboard frames, got {len(objpoints)}")

    rms, K, dist, _, _ = cv2.calibrateCamera(objpoints, imgpoints, img_shape, None, None)
    return {
        "width": int(img_shape[0]),
        "height": int(img_shape[1]),
        "rms_reproj_px": round(float(rms), 4),
        "K": K.tolist(),
        "dist": dist.ravel().tolist(),
        "n_frames_used": len(objpoints),
    }


def main():
    ap = argparse.ArgumentParser(description="Calibrate Sourccey 20x20 mm wrist camera intrinsics")
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--cols", type=int, default=9, help="Inner chessboard corners (columns)")
    ap.add_argument("--rows", type=int, default=6, help="Inner chessboard corners (rows)")
    ap.add_argument("--square-mm", type=float, default=20.0)
    ap.add_argument("--target-frames", type=int, default=20)
    ap.add_argument("--out", default="camera_calib.json")
    a = ap.parse_args()

    cap = cv2.VideoCapture(a.camera)
    if not cap.isOpened():
        raise SystemExit(f"Could not open camera {a.camera}")

    frames = []
    print(f"Show a {a.cols}x{a.rows} chessboard at varied angles. Press SPACE to capture ({a.target_frames} needed), Q to finish.")
    while len(frames) < a.target_frames:
        ok, fr = cap.read()
        if not ok:
            continue
        disp = fr.copy()
        cv2.putText(disp, f"Captured: {len(frames)}/{a.target_frames} (SPACE=capture, Q=done)",
                    (16, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 0), 2)
        cv2.imshow("Sourccey Camera Calibration", disp)
        k = cv2.waitKey(1) & 0xFF
        if k == ord(" "):
            frames.append(fr.copy())
            print(f"Captured frame {len(frames)}/{a.target_frames}")
        elif k in (ord("q"), 27):
            break

    cap.release()
    cv2.destroyAllWindows()
    cal = calibrate_from_frames(frames, a.cols, a.rows, a.square_mm / 1000.0)
    Path(a.out).write_text(json.dumps(cal, indent=2), encoding="utf-8")
    print(f"Saved {a.out} (RMS reprojection error: {cal['rms_reproj_px']} px)")


if __name__ == "__main__":
    main()
