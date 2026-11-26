"""Align the servo clock with the tracking video using the finger's own motion.
ArUco markers (ID 0 on the finger, ID 1 on the base; see make_markers.py) appear in the video; the pixel distance between them varies with the angle.
Correlates that signal with the servo q and returns the delay. Convention: returns `lag` such that corrected_servo_time = t_servo - lag."""
import numpy as np, cv2

def video_opening_signal(video_path, id_finger=0, id_base=1, dictionary=cv2.aruco.DICT_4X4_50, frame_times=None):
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(dictionary), cv2.aruco.DetectorParameters())
    cap = cv2.VideoCapture(video_path); fps = cap.get(cv2.CAP_PROP_FPS) or 30.0; t, d, i = [], [], 0
    while True:
        ok, fr = cap.read()
        if not ok: break
        corners, ids, _ = det.detectMarkers(fr)
        if ids is not None:
            c = {int(k): v.reshape(4, 2).mean(0) for k, v in zip(ids.ravel(), corners)}
            if id_finger in c and id_base in c:
                t.append(frame_times[i] if frame_times is not None else i / fps); d.append(np.linalg.norm(c[id_finger] - c[id_base]))
        i += 1
    return np.array(t), np.array(d)

def estimate_lag(ta, a, tb, b, max_lag=1.0, fs=200.0):
    """b(t) ~ a(t - lag). ta/tb in seconds. Returns lag in s (b DELAYED relative to a => lag > 0)."""
    t0, t1 = max(ta[0], tb[0]) + max_lag, min(ta[-1], tb[-1]) - max_lag
    if t1 <= t0: raise ValueError("signals too short for the search window")
    g = np.arange(t0, t1, 1 / fs)
    A, B = np.interp(g, ta, a), None
    n = int(max_lag * fs); best = None; scores = []
    Bfun = lambda tt: np.interp(tt, tb, b)
    Az = (A - A.mean()) / (A.std() + 1e-12)
    lags = np.arange(-n, n + 1)
    for k in lags:
        Bk = Bfun(g + k / fs); Bz = (Bk - Bk.mean()) / (Bk.std() + 1e-12); scores.append(float((Az * Bz).mean()))
    scores = np.array(scores); i = int(scores.argmax())
    if 0 < i < len(scores) - 1:                                     # parabolic refinement
        y0, y1, y2 = scores[i - 1:i + 2]; den = y0 - 2 * y1 + y2; frac = 0.5 * (y0 - y2) / den if den else 0.0
    else: frac = 0.0
    return (lags[i] + frac) / fs, float(scores[i])
