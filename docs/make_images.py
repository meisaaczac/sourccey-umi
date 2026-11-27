"""Generate the step-by-step images (docs/img/*.png) from the CAD and the real Sourccey parts. python docs/make_images.py"""
import sys, math
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]; sys.path[:0] = [str(ROOT / "tools"), str(ROOT / "cad")]
import numpy as np, trimesh, cadquery as cq, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from trimesh.transformations import rotation_matrix as Rm
import common as C, umi_cad as U
OUT = ROOT / "docs" / "img"; OUT.mkdir(exist_ok=True)
real = C.load_pivot_frame("L", ("base", "finger", "servo")); B, F, SV = real["base"], real["finger"], real["servo"]
def tm(wp, n): cq.exporters.export(wp, f"/tmp/_i_{n}.stl", tolerance=0.04, angularTolerance=0.12); return trimesh.load(f"/tmp/_i_{n}.stl")
L, G = tm(U.make_lever(), "l"), tm(U.make_grip(), "g")
COL = dict(base=(0.22, 0.38, 0.75), finger=(0.95, 0.5, 0.15), lever=(0.15, 0.65, 0.35), grip=(0.38, 0.40, 0.44), servo=(0.75, 0.75, 0.8), hand=(0.93, 0.74, 0.62), red=(0.85, 0.15, 0.15))
def rot(m, q): mm = m.copy(); mm.apply_transform(Rm(np.radians(-q), [0, 1, 0])); return mm
def shift(m, d): mm = m.copy(); mm.apply_translation(d); return mm
def t2(ax, x, y, s, col='#222', size=11, **kw): ax.text2D(x, y, s, transform=ax.transAxes, color=col, fontsize=size, fontweight='bold', zorder=30, **kw)
def draw(ax, m, col, alpha=1.0, z=1):
    tri = m.vertices[m.faces]; n = m.face_normals; Lv = np.array([-0.4, -0.6, 0.7]); Lv /= np.linalg.norm(Lv); sh = 0.32 + 0.68 * np.clip(n @ Lv, 0, 1)
    c = np.clip(np.c_[sh * col[0], sh * col[1], sh * col[2]], 0, 1); ax.add_collection3d(Poly3DCollection(tri, facecolors=np.c_[c, np.full(len(c), alpha)], edgecolors="none", zorder=z))
def view(ax, xl, yl, zl, az, el, zoom=1.0):
    ax.set_xlim(*xl); ax.set_ylim(*yl); ax.set_zlim(*zl); ax.set_box_aspect((xl[1] - xl[0], yl[1] - yl[0], zl[1] - zl[0]), zoom=zoom); ax.view_init(elev=el, azim=az); ax.set_axis_off()
def line(ax, p, q, col="r", ls="--", lw=1.6): ax.plot([p[0], q[0]], [p[1], q[1]], [p[2], q[2]], color=col, ls=ls, lw=lw)
def txt(ax, p, s, col="#222", size=10, **kw): ax.text(p[0], p[1], p[2], s, color=col, fontsize=size, fontweight="bold", **kw)
def fig1(title, caption, n=1, size=(9.5, 6.4)):
    f = plt.figure(figsize=size, facecolor="white"); f.suptitle(title, fontsize=15, fontweight="bold", y=0.97); f.text(0.5, 0.03, caption, ha="center", fontsize=11, color="#333", wrap=True); return f
def capsule(p0, p1, r):
    p0, p1 = np.array(p0, float), np.array(p1, float); h = np.linalg.norm(p1 - p0); c = trimesh.creation.capsule(height=h, radius=r)
    T = trimesh.geometry.align_vectors([0, 0, 1], (p1 - p0) / h); c.apply_transform(T); c.apply_translation((p0 + p1) / 2); return c

HOLES = [(U.XF, y, z) for (y, z) in U.HOLES]

# 01 parts
f = fig1("Step 1: print 4 parts", "Base and finger come from Sourccey (tools/prepare_sourccey_parts.py). Grip and lever are from this repo (cad/stl). PETG, 0.2 mm, 4 perimeters, 40%. Already oriented for printing.", size=(10, 7.2))
files = [("1  Sourccey base", ROOT / "cad" / "stl_sourccey" / "base_sourccey_gripper_LEFT.stl", "base"), ("2  Sourccey finger", ROOT / "cad" / "stl_sourccey" / "finger_sourccey_gripper_LEFT.stl", "finger"),
         ("3  pistol grip (this repo)", ROOT / "cad" / "stl" / "pistol_grip_LEFT.stl", "grip"), ("4  trigger lever (this repo)", ROOT / "cad" / "stl" / "trigger_lever_LEFT.stl", "lever")]
for i, (name, path, k) in enumerate(files):
    ax = f.add_subplot(2, 2, i + 1, projection="3d"); m = trimesh.load(path); c = m.bounds.mean(0); r = (m.bounds[1] - m.bounds[0]).max() / 2
    draw(ax, m, COL[k]); view(ax, (c[0] - r, c[0] + r), (c[1] - r, c[1] + r), (c[2] - r, c[2] + r), -55, 22, 1.25); ax.set_title(name, fontsize=12, fontweight="bold")
f.savefig(OUT / "01_parts.png", dpi=105, bbox_inches="tight"); plt.close(f)

# 02 inserts
f = fig1("Step 2: 5 M3 heat-set inserts in the grip", "Heat the soldering iron (~220 °C) and press slowly and straight until flush with the face. The 5 holes (cross) are marked in red.")
ax = f.add_subplot(111, projection="3d"); ax.computed_zorder = False; draw(ax, G, COL["grip"], 1.0, 1)
for (x, y, z) in HOLES:
    t = np.linspace(0, 2 * np.pi, 30); pts = np.c_[np.full(30, x + 2.0), y + 2.8 * np.cos(t), z + 2.8 * np.sin(t)]
    ax.add_collection3d(Poly3DCollection([pts], facecolors=[(0.9, 0.1, 0.1, 1)], edgecolors="white", linewidths=1.2, zorder=10))
t2(ax, 0.5, 0.93, "5x M3 insert  (hole Ø4.2 mm, 6 mm deep)", "red", 12, ha="center")
view(ax, (-70, -20), (-22, 22), (-42, 8), 0, 0, 1.2); f.savefig(OUT / "02_inserts.png", dpi=105, bbox_inches="tight"); plt.close(f)

# 03 grip on the base
f = fig1("Step 3: grip onto the Sourccey base", "5x M3x20 screws, from INSIDE the base (head in the 6.2 mm counterbore) into the grip. If the servo blocks access, do this before installing it.")
ax = f.add_subplot(111, projection="3d"); draw(ax, B, COL["base"]); draw(ax, shift(G, (-45, 0, 0)), COL["grip"])
for (x, y, z) in HOLES: line(ax, (x + 12, y, z), (x - 45 - 6, y, z), "red")
t2(ax, 0.62, 0.8, "5x M3x20", "red", 13); view(ax, (-120, 40), (-60, 60), (-135, 25), -55, 14, 1.25); f.savefig(OUT / "03_grip_on_base.png", dpi=105, bbox_inches="tight"); plt.close(f)

# 04 lever on the finger
f = fig1("Step 4: lever on the -y side of the finger", "Uses the 4-hole cross the real finger already has: 4x M3x16 (if screws are already there, swap them for ones 6 mm longer). Tighten and turn by hand: no play allowed.")
ax = f.add_subplot(111, projection="3d"); draw(ax, B, COL["base"], 0.25); draw(ax, F, COL["finger"]); draw(ax, shift(L, (0, -34, 0)), COL["lever"])
for dx, dz in ((7.1, 0), (-7.1, 0), (0, 7.1), (0, -7.1)): line(ax, (dx, -70, dz), (dx, -26, dz), "red")
t2(ax, 0.5, 0.88, "4x M3x16 (red lines)", "red", 13, ha="center"); view(ax, (-30, 90), (-90, 60), (-70, 40), -50, 16, 1.3); f.savefig(OUT / "04_lever_on_finger.png", dpi=105, bbox_inches="tight"); plt.close(f)

# 05 assembled gripper
f = fig1("Step 5: assemble the gripper as in the Sourccey manual", "STS3215 servo + horn + finger exactly as in the Sourccey Assembly Manual (vulcan-forge repo). The lever goes on with the finger. The servo (light gray) sits inside the base.")
ax = f.add_subplot(111, projection="3d"); draw(ax, B, COL["base"], 0.55); draw(ax, SV, COL["servo"]); draw(ax, rot(F, 20), COL["finger"]); draw(ax, rot(L, 20), COL["lever"]); draw(ax, G, COL["grip"])
view(ax, (-70, 100), (-60, 60), (-140, 40), -58, 16, 1.5); f.savefig(OUT / "05_assembled_gripper.png", dpi=105, bbox_inches="tight"); plt.close(f)

# 06 spring and pin
f = fig1("Step 6: pin and spring", "An M3x70 pin + nut goes through the beam at (x=-42, z=-26). The spring runs from the pin (inside the channel) to the hole in the lever tail; it stretches when closing and reopens the gripper.", size=(15, 5.6))
TAIL = np.array(U.TAIL)
for k, q in enumerate((-5, 30, 60)):
    ax = f.add_subplot(1, 3, k + 1, projection="3d"); draw(ax, B, COL["base"], 0.3); draw(ax, G, COL["grip"]); draw(ax, rot(L, q), COL["lever"]); draw(ax, rot(F, q), COL["finger"], 0.5)
    th = math.radians(-q); h = np.array([TAIL[0] * math.cos(th) + TAIL[1] * math.sin(th), -TAIL[0] * math.sin(th) + TAIL[1] * math.cos(th)]); yc = -35.3
    line(ax, (h[0], yc, h[1]), (-42, yc, -26), "red", "-", 3); line(ax, (-42, -52, -26), (-42, 12, -26), "#1f4fd8", "--", 2)
    ax.set_title(f"q = {q}°   spring = {math.hypot(h[0] + 42, h[1] + 26):.0f} mm", fontsize=12); view(ax, (-85, 45), (-45, 20), (-70, 45), -90, 0, 1.15)
f.savefig(OUT / "06_spring_and_pin.png", dpi=105, bbox_inches="tight"); plt.close(f)

# 07 camera
f = fig1("Step 7: the Sourccey 20x20 mm camera", "The base's side wing has a Ø25 mm opening aimed at the jaws (the same viewpoint as the robot). The 20x20 board sits behind the opening; use the mount your robot already uses.")
ax = f.add_subplot(111, projection="3d"); ax.computed_zorder = False; draw(ax, B, COL["base"]); draw(ax, rot(F, 30), COL["finger"], 0.8); draw(ax, G, COL["grip"], 0.4)
Cc = np.array([-11.5, 50.7, -17.5]); n = np.array([0.93, -0.37, 0.0]); n /= np.linalg.norm(n); u = np.array([0.37, 0.93, 0.0]); v = np.array([0, 0, 1.0])
t = np.linspace(0, 2 * np.pi, 40); ring = Cc + 12.5 * np.outer(np.cos(t), u) + 12.5 * np.outer(np.sin(t), v); ax.plot(ring[:, 0], ring[:, 1], ring[:, 2], color="red", lw=3, zorder=20)
sq = np.array([Cc - n * 4 + s1 * 10 * u + s2 * 10 * v for s1, s2 in ((-1, -1), (1, -1), (1, 1), (-1, 1), (-1, -1))]); ax.plot(sq[:, 0], sq[:, 1], sq[:, 2], color="#00a000", lw=3, zorder=20)
for tx in ((45, -6), (65, 0), (75, 6)): line(ax, Cc, (tx[0], tx[1], -22), "#e0b000", ":", 1.5)
t2(ax, 0.02, 0.92, "20x20 camera (green square)", "#00a000", 12, ha="left"); t2(ax, 0.02, 0.86, "Ø25 mm opening (red circle)", "red", 12, ha="left"); t2(ax, 0.02, 0.80, "yellow lines: line of sight to the jaws", "#a07800", 10, ha="left")
view(ax, (-60, 100), (-60, 70), (-70, 20), -40, 40, 1.45); f.savefig(OUT / "07_camera.png", dpi=105, bbox_inches="tight"); plt.close(f)

# 08 how to hold
f = fig1("Step 8: how to hold it", "Illustrative sketch. Palm and 3 fingers in the grip grooves; the index finger sits in front of the trigger and pulls back (closes). Releasing reopens.")
ax = f.add_subplot(111, projection="3d"); q = 30; draw(ax, B, COL["base"], 0.6); draw(ax, G, COL["grip"]); draw(ax, rot(L, q), COL["lever"]); draw(ax, rot(F, q), COL["finger"], 0.8)
for z in (-70, -90, -110): draw(ax, capsule((-31, -52, z), (-31, -22, z), 7), COL["hand"], 0.9)
th = math.radians(-q); pc = np.array([8 * math.cos(th) + (-41.0) * math.sin(th), -37.0, -8 * math.sin(th) + (-41.0) * math.cos(th)]); 
pc = np.array([pc[0], -37.0, pc[2]]); draw(ax, capsule((-40, -37, -38), (pc[0] + 3, -37, pc[2]), 7.5), COL["hand"], 0.9)
draw(ax, shift(trimesh.creation.box((14, 34, 70)), (-74, -37, -80)), COL["hand"], 0.9)
t2(ax, 0.02, 0.92, "index finger: in front of the trigger, pulls back", size=11, ha="left"); t2(ax, 0.02, 0.86, "3 fingers: in the grip grooves", size=11, ha="left"); t2(ax, 0.02, 0.80, "palm: behind the grip (block)", size=11, ha="left")
view(ax, (-95, 60), (-70, 45), (-140, 20), -150, 14, 1.4); f.savefig(OUT / "08_how_to_hold.png", dpi=105, bbox_inches="tight"); plt.close(f)

# 09 operation
f = fig1("Step 9: operation", "Trigger released = gripper open (q up to 60°). Pull = closes (q = -5°). The servo, torque off, reads the angle: it is the Sourccey `gripper` joint itself.", size=(14, 5.6))
for k, (q, tt) in enumerate(((60, "released: open, q = 60°"), (30, "halfway, q = 30°"), (-5, "pulled: closed, q = -5°"))):
    ax = f.add_subplot(1, 3, k + 1, projection="3d"); draw(ax, B, COL["base"]); draw(ax, G, COL["grip"]); draw(ax, rot(L, q), COL["lever"]); draw(ax, rot(F, q), COL["finger"]); ax.set_title(tt, fontsize=12)
    view(ax, (-75, 100), (-60, 60), (-140, 40), -62, 14, 1.35)
f.savefig(OUT / "09_operation.png", dpi=105, bbox_inches="tight"); plt.close(f)
print("images:", sorted(p.name for p in OUT.glob("*.png")))
