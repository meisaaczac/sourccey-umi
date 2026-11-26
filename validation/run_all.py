"""Run EVERY validation that can be done without the physical part and write validation/RESULTS.md.
  python validation/run_all.py          (needs internet the first time: downloads the real Sourccey parts)
Each check has a numeric criterion. Exit code 1 if anything fails."""
import sys, math, json, subprocess, time
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]; sys.path[:0] = [str(ROOT / "tools"), str(ROOT / "cad")]
import numpy as np, trimesh, cadquery as cq
from scipy.spatial import cKDTree
from scipy import ndimage as ndi
from shapely.geometry import Polygon
from trimesh.transformations import rotation_matrix as Rm
import common as C, umi_cad as U

rows = []; T0 = time.time()
def check(name, ok, detail, criterion): rows.append((name, "PASS" if ok else "FAIL", detail, criterion)); print(("PASS" if ok else "FAIL"), name, "|", detail)
def tm(wp, n): cq.exporters.export(wp, f"/tmp/_v_{n}.stl", tolerance=0.03, angularTolerance=0.1); return trimesh.load(f"/tmp/_v_{n}.stl")

real = C.load_pivot_frame("L", ("base", "finger", "servo")); B, F, SV = real["base"], real["finger"], real["servo"]
L, G = tm(U.make_lever(), "lever"), tm(U.make_grip(), "grip")
check("My parts are closed meshes (watertight)", L.is_watertight and G.is_watertight, f"lever={L.is_watertight}, grip={G.is_watertight}", "both watertight")

# 1) collision sweep over the whole travel (q = -5 to 65 deg) against the real parts
bt, gt = cKDTree(B.sample(300000)), cKDTree(G.sample(150000)); lp, fp = L.sample(10000), F.sample(10000)
dlb, dlg, dfg, stop = [], [], [], None
for q in range(-5, 66, 1):
    M = Rm(np.radians(-q), [0, 1, 0])[:3, :3]; l = (M @ lp.T).T; f = (M @ fp.T).T
    dlb.append(bt.query(l)[0].min()); dlg.append(gt.query(l)[0].min()); dfg.append(gt.query(f)[0].min())
    if stop is None and dlg[-1] < 0.3: stop = q
check("Lever never touches the real base (q from -5 to 65)", min(dlb) >= 3.0, f"minimum clearance {min(dlb):.2f} mm", ">= 3 mm")
check("Real finger never touches the grip", min(dfg) >= 10, f"minimum clearance {min(dfg):.1f} mm", ">= 10 mm")
check("Trigger clears the grip when fully squeezed (q=-5)", dlg[0] >= 3.0, f"{dlg[0]:.2f} mm", ">= 3 mm")
check("Open stop lands at the robot's limit (60 deg)", stop is not None and 57 <= stop <= 62, f"first contact at q={stop}", "57 to 62 deg")

# 2) spring: always pushes open, length is monotonic
TAIL, ANCH = np.array(U.TAIL), np.array([-42.0, -26.0])
def hook(q): th = math.radians(-q); x, z = TAIL; return np.array([x * math.cos(th) + z * math.sin(th), -x * math.sin(th) + z * math.cos(th)])
tau, ln = [], []
for q in range(-5, 61):
    h = hook(q); d = ANCH - h; n = np.linalg.norm(d); Fv = d / n; tau.append(h[1] * Fv[0] - h[0] * Fv[1]); ln.append(n)
check("Spring gives OPENING torque over the whole travel", max(tau) < 0, f"minimum lever arm {-max(tau):.1f} mm", "always opening")
check("Spring length shrinks monotonically as the gripper opens", all(np.diff(ln) <= 1e-6), f"{ln[0]:.1f} mm (closed) to {ln[-1]:.1f} mm (open)", "monotonic")
L0, k = 15.0, 0.15; f_open, f_closed = k * (ln[-1] - L0), k * (ln[0] - L0); arm_o, arm_c = -tau[-1], -tau[0]
rows.append(("Spring budget (informative)", "INFO", f"spring L0={L0:.0f} mm, k={k} N/mm: opening torque {f_open*arm_o/1000:.3f} N·m (open) to {f_closed*arm_c/1000:.3f} N·m (closed); trigger effort when closing ~{f_closed*arm_c/1000/0.044:.1f} N. It only reopens by itself if the unpowered-servo friction is LOWER than the open-end torque, which is why the velcro strap is required until bench tests 4/5 pass", "not CAD-dependent"))

# 3) holes: finger cross (-y lug) vs lever holes
s = F.section(plane_origin=[0, -28.0, 0], plane_normal=[0, 1, 0]); p2, Tm = s.to_2D(); fh = []
for poly in p2.polygons_full:
    for r in poly.interiors:
        pr = Polygon(r); c = pr.centroid; c3 = trimesh.transform_points([[c.x, c.y, 0]], Tm)[0]; d = 2 * math.sqrt(pr.area / math.pi)
        if d < 4: fh.append((c3[0], c3[2]))
dev = [min(math.hypot(a - x, b - z) for (x, z) in fh) for (a, b) in ((7.1, 0), (-7.1, 0), (0, 7.1), (0, -7.1))] if len(fh) >= 4 else [99]
check("Lever holes match the real finger's 4-hole cross", max(dev) < 0.3, f"{len(fh)} holes of 3 mm found in the finger; max offset {max(dev):.2f} mm", "< 0.3 mm")
s = B.section(plane_origin=[-27.0, 0, 0], plane_normal=[1, 0, 0]); p2, Tm = s.to_2D(); bh = []
for poly in p2.polygons_full:
    for r in poly.interiors:
        pr = Polygon(r); c = pr.centroid; c3 = trimesh.transform_points([[c.x, c.y, 0]], Tm)[0]; bh.append((round(c3[1], 1), round(c3[2], 1)))
dev2 = [min(math.hypot(a - y, b - z) for (y, z) in bh) for (a, b) in U.HOLES] if bh else [99]
check("Grip holes match the real base's rear interface", max(dev2) < 0.3, f"{len(bh)} holes found; max offset {max(dev2):.2f} mm", "< 0.3 mm")

# 4) right/left mirror
R = C.load_pivot_frame("R"); worst = 0
for k_ in ("base", "finger"):
    mr = C.mirror_y(real[k_]); a = cKDTree(R[k_].sample(120000)); b = cKDTree(mr.sample(120000))
    worst = max(worst, np.percentile(a.query(mr.sample(30000))[0], 95), np.percentile(b.query(R[k_].sample(30000))[0], 95))
check("Sourccey's right gripper is a mirror of the left", worst < 1.2, f"difference p95 {worst:.2f} mm", "< 1.2 mm")

# 5) servo and camera
st = cKDTree(SV.sample(100000)); ds = min(st.query(((Rm(np.radians(-q), [0, 1, 0])[:3, :3] @ lp.T).T))[0].min() for q in range(-5, 61, 5)); dg = st.query(G.sample(20000))[0].min()
check("Lever and grip do not touch the real servo", min(ds, dg) >= 5, f"lever {ds:.1f} mm, grip {dg:.1f} mm", ">= 5 mm")
Cam = np.array([-11.5, 50.7, -17.5])        # center of the 25 mm camera opening (from the CAD)
tg = np.array([[x, y, -24 + (x - 45) * 0.25] for x in (45, 55, 65, 75) for y in (-6, 0, 6)], float)
def blocked(meshes):
    d = tg - Cam; dist = np.linalg.norm(d, axis=1); dirs = d / dist[:, None]; bl = np.zeros(len(tg), bool)
    for m in meshes:
        loc, idx, _ = m.ray.intersects_location(np.tile(Cam, (len(tg), 1)), dirs, multiple_hits=True)
        if len(idx): hit = np.linalg.norm(loc - Cam, axis=1) < dist[idx] - 1.0; bl[np.unique(idx[hit])] = True
    return int(bl.sum())
extra = 0
for q in (10, 30, 50):
    M = Rm(np.radians(-q), [0, 1, 0]); Fq, Lq = F.copy(), L.copy(); Fq.apply_transform(M); Lq.apply_transform(M)
    extra = max(extra, blocked([B, Fq, Lq, G]) - blocked([B, Fq]))
check("My parts do not block the camera's view of the jaw gap", extra == 0, f"extra blocked targets: {extra} (of 12, q=10/30/50)", "0")

# 6) manufacturing
vol = {}
for n, m in (("lever", L), ("grip", G)):
    a = m.voxelized(1.0).fill().matrix; op = ndi.binary_opening(a, structure=ndi.generate_binary_structure(3, 3)); vol[n] = 100 * (a.sum() - op.sum()) / a.sum()
check("Almost nothing thinner than ~2 mm", max(vol.values()) < 1.0, f"volume in thin features: lever {vol['lever']:.2f}%, grip {vol['grip']:.2f}%", "< 1% of volume")
mass = {n: m.volume * 1.27e-3 * 0.45 for n, m in (("lever", L), ("grip", G), ("real base", B), ("real finger", F))}
rows.append(("Printed mass estimate (PETG, 40% infill)", "INFO", ", ".join(f"{k_} {v:.0f} g" for k_, v in mass.items()) + f"; total {sum(mass.values()):.0f} g (+ servo ~55 g, camera and screws)", "estimate"))

# 7) strength (hand calculation)
F_pad, r_pad, w, t = 40.0, 44.0, 14.0, 6.0               # strong finger force (N), pad radius (mm), arm width and thickness (mm)
M_hub = F_pad * r_pad; sig = (F_pad * 35.0) / (t * w * w / 6); per_screw = M_hub / (4 * 7.1); bear = per_screw / (3.0 * t)
check("Lever withstands 40 N on the trigger (hand calculation)", sig < 15 and bear < 10, f"arm stress {sig:.1f} MPa (PETG ~40+), screw bearing {bear:.1f} MPa, load per screw {per_screw:.0f} N", "stress < 15 MPa, bearing < 10 MPa")
Mi = 0.4; Tn = Mi / (2 * 0.0071) / 2
check("Grip interface withstands 0.4 N·m (hand calculation)", Tn < 100, f"tension per screw ~{Tn:.0f} N (an M3 insert in PETG holds hundreds of N)", "< 100 N")

# 8) software
for name in ("test_software.py", "test_pipeline.py"):
    r = subprocess.run([sys.executable, str(ROOT / "software" / name)], capture_output=True, text=True, cwd=ROOT / "software")
    check(f"software/{name}", r.returncode == 0, (r.stdout.strip().splitlines() or ["?"])[-1], "exit code 0")

fails = [r for r in rows if r[1] == "FAIL"]
md = ["# Validation results (generated by `python validation/run_all.py`)", "", f"Run time: {time.time()-T0:.0f} s. **{len([r for r in rows if r[1]=='PASS'])} passed, {len(fails)} failed.**",
      "This covers only what can be proven on a computer, against the REAL Sourccey parts (STL from Vulcan's URDF). Nothing here replaces printing and testing: see `BENCH_CHECKLIST.md`.", "",
      "| Check | Result | Detail | Criterion |", "|---|---|---|---|"] + [f"| {a} | {b} | {c} | {d} |" for a, b, c, d in rows]
(ROOT / "validation" / "RESULTS.md").write_text("\n".join(md) + "\n"); sys.exit(1 if fails else 0)
