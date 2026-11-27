"""Render clean, Vulcan-Robotics Sourccey Assembly Manual style step illustrations.

Each figure uses MuJoCo with exact CAD meshes, soft technical-drawing shading
+ crisp silhouette/crease edge ink (Fusion 360 / SolidWorks drawing aesthetic),
boxed fastener callout pills (e.g. "(5) M3x20mm"), and centered figure captions.

Generates docs/img/01_parts.png through 07_camera.png and 09_operation.png.
"""
import math
import pathlib
import sys

import cv2
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "docs"))
from render_demo_video import ensure_assets, UMI_MESH_DIR, get_font

IMG_DIR = ROOT / "docs" / "img"
HOLES_YZ = [(-0.0004, -0.0150), (-0.0004, -0.0079), (-0.0004, -0.0221), (0.0067, -0.0150), (-0.0074, -0.0150)]
LEVER_HOLES_XZ = [(0.0071, 0.0), (-0.0071, 0.0), (0.0, 0.0071), (0.0, -0.0071)]


def _render_tech_view(xml: str, w: int = 1280, h: int = 760, qpos=None) -> Image.Image:
    """Render MuJoCo scene and apply clean technical-drawing silhouette & crease ink."""
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    if qpos is not None:
        for i, val in enumerate(qpos):
            d.qpos[i] = val
    mujoco.mj_forward(m, d)

    r = mujoco.Renderer(m, h, w)
    r.update_scene(d, camera="cam")
    rgb = r.render()

    r.enable_depth_rendering()
    r.update_scene(d, camera="cam")
    depth = r.render()
    r.close()

    # Crisp silhouette + depth-discontinuity edges (SolidWorks / Sourccey manual style)
    valid = depth < (depth.max() * 0.995)
    d_norm = np.where(valid, depth, np.nan)
    if np.any(valid):
        dmin, dmax = np.nanmin(d_norm), np.nanmax(d_norm)
        d_u8 = np.where(valid, np.clip((depth - dmin) / max(dmax - dmin, 1e-6) * 255.0, 0, 255), 255).astype(np.uint8)
    else:
        d_u8 = np.full((h, w), 255, dtype=np.uint8)

    # Combine depth edges + subtle RGB crease edges
    edges_d = cv2.Canny(d_u8, 18, 55)
    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    edges_g = cv2.Canny(gray, 40, 110)
    # Keep background pure white
    edges = np.where(valid | (cv2.dilate(valid.astype(np.uint8), np.ones((3, 3), np.uint8)) > 0),
                     np.maximum(edges_d, (edges_g * 0.65).astype(np.uint8)), 0)

    out = rgb.astype(np.float32)
    # Set background cleanly to pure white
    out[~valid] = 255.0
    # Dark slate ink on edges
    ink = (edges.astype(np.float32) / 255.0)[..., None]
    out = out * (1.0 - 0.72 * ink) + np.array([28.0, 34.0, 44.0]) * (0.72 * ink)
    return Image.fromarray(np.clip(out, 0, 255).astype(np.uint8))


def ensure_electronics_cad():
    """Center and copy real open-source electronics CAD STLs (from Grabette Apache-2.0) into UMI_MESH_DIR."""
    import trimesh
    grab_root = ROOT / ".cache" / "grabette"
    specs = [
        ("pi_zero_cad.stl", "rasp_pi_0_2w__configuration_default.stl", 1.0, (90, 0, 0)),
        ("battery_cad.stl", "pisugar3plus.stl", 1.0, (90, 0, 0)),
        ("hat_cad.stl", "ase01187_elec_rpi_hat.stl", 1.0, (0, 0, 0)),
        ("imu_cad.stl", "sensor.stl", 1.0, (0, 0, 0)),
        ("button_cad.stl", "Button.stl", 1.0, (0, 0, 0)),
        ("cam_cad.stl", "arducam_fisheye.stl", 20.0 / 36.0, (0, 0, 0)),
    ]
    for dst_name, src_name, scale, rot_deg in specs:
        dst = UMI_MESH_DIR / dst_name
        matches = list(grab_root.rglob(src_name))
        if not matches:
            raise FileNotFoundError(f"Missing real CAD mesh {src_name} in {grab_root}")
        m = trimesh.load(matches[0])
        if scale != 1.0:
            m.apply_scale(scale)
        if any(rot_deg):
            from scipy.spatial.transform import Rotation as Rot
            T = np.eye(4)
            T[:3, :3] = Rot.from_euler("xyz", rot_deg, degrees=True).as_matrix()
            m.apply_transform(T)
        # Center XY and rest bottom at z = 0
        b = m.bounds
        m.apply_translation([-(b[0][0] + b[1][0]) / 2.0, -(b[0][1] + b[1][1]) / 2.0, -b[0][2]])
        m.export(dst)


def _base_header(w=1280, h=760, fovy=32):
    mesh_dir = str(UMI_MESH_DIR).replace("\\", "/")
    return f"""
<mujoco model="manual_step">
  <compiler angle="radian" meshdir="{mesh_dir}"/>
  <visual>
    <global offwidth="{w}" offheight="{h}" fovy="{fovy}"/>
    <headlight ambient="0.52 0.53 0.55" diffuse="0.48 0.48 0.48" specular="0.12 0.12 0.12"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="flat" rgb1="1 1 1" rgb2="1 1 1" width="16" height="16"/>
    <!-- Neutral light-gray for context parts, crisp accent colors for active step parts -->
    <material name="ctx_mat" rgba="0.84 0.86 0.89 1" roughness="0.55" specular="0.12"/>
    <material name="base_mat" rgba="0.22 0.44 0.82 1" roughness="0.45" specular="0.20"/>
    <material name="finger_mat" rgba="0.96 0.52 0.18 1" roughness="0.45" specular="0.20"/>
    <material name="lever_mat" rgba="0.18 0.64 0.38 1" roughness="0.45" specular="0.20"/>
    <material name="grip_mat" rgba="0.34 0.37 0.42 1" roughness="0.55" specular="0.15"/>
    <material name="servo_mat" rgba="0.22 0.24 0.28 1" roughness="0.40" specular="0.25"/>
    <material name="brass_mat" rgba="0.86 0.66 0.22 1" roughness="0.25" specular="0.65"/>
    <material name="steel_mat" rgba="0.62 0.66 0.72 1" roughness="0.25" specular="0.65"/>
    <material name="spring_mat" rgba="0.85 0.18 0.18 1" roughness="0.35" specular="0.30"/>
    <material name="cam_mat" rgba="0.18 0.20 0.24 1" roughness="0.35" specular="0.35"/>
    <material name="lens_mat" rgba="0.06 0.12 0.28 1" roughness="0.10" specular="0.90"/>
    <material name="pcb_green" rgba="0.16 0.56 0.30 1" roughness="0.40" specular="0.25"/>
    <material name="pcb_purple" rgba="0.44 0.24 0.76 1" roughness="0.40" specular="0.25"/>
    <material name="pcb_orange" rgba="0.88 0.44 0.14 1" roughness="0.40" specular="0.25"/>
    <material name="batt_mat" rgba="0.82 0.22 0.20 1" roughness="0.42" specular="0.25"/>
    <material name="btn_mat" rgba="0.92 0.72 0.10 1" roughness="0.40" specular="0.25"/>
    <material name="guide_mat" rgba="0.35 0.40 0.48 0.9"/>

    <mesh name="umi_base" file="umi_base.stl"/>
    <mesh name="umi_finger" file="umi_finger.stl"/>
    <mesh name="umi_servo" file="umi_servo.stl"/>
    <mesh name="umi_grip" file="umi_grip.stl"/>
    <mesh name="umi_lever" file="umi_lever.stl"/>
    <mesh name="pi_zero_cad" file="pi_zero_cad.stl"/>
    <mesh name="battery_cad" file="battery_cad.stl"/>
    <mesh name="hat_cad" file="hat_cad.stl"/>
    <mesh name="imu_cad" file="imu_cad.stl"/>
    <mesh name="button_cad" file="button_cad.stl"/>
    <mesh name="cam_cad" file="cam_cad.stl"/>
  </asset>
  <worldbody>
    <light pos="0.5 -0.6 0.8" dir="-0.4 0.5 -0.7" diffuse="0.52 0.52 0.52" specular="0.15 0.15 0.15" castshadow="false"/>
    <light pos="-0.5 0.5 0.6" dir="0.4 -0.4 -0.6" diffuse="0.30 0.30 0.32" specular="0.08 0.08 0.08" castshadow="false"/>
"""


def _frame_figure(canvas: Image.Image, figure_caption: str, callouts: list[str] = ()) -> Image.Image:
    """Wrap rendered image with Sourccey-manual style boxed callout(s) + centered Figure caption."""
    W, H = canvas.size
    out = Image.new("RGB", (W, H + 56), (255, 255, 255))
    out.paste(canvas, (0, 0))
    draw = ImageDraw.Draw(out)

    # Boxed hardware callout at top-left (exact Sourccey Manual Page 149 style!)
    if callouts:
        font_c = get_font(22, bold=True)
        pad_x, pad_y = 16, 10
        line_h = 30
        max_w = max(int(draw.textlength(f"• {c}", font=font_c)) for c in callouts)
        bx0, by0 = 28, 24
        bx1, by1 = bx0 + max_w + pad_x * 2, by0 + len(callouts) * line_h + pad_y * 2 - 4
        draw.rectangle([bx0, by0, bx1, by1], fill=(255, 255, 255), outline=(45, 55, 72), width=2)
        for i, c in enumerate(callouts):
            draw.text((bx0 + pad_x, by0 + pad_y + i * line_h), f"• {c}", fill=(20, 28, 40), font=font_c)

    # Centered caption at bottom (Sourccey Manual style: "Figure X.Y - Title")
    font_cap = get_font(20, bold=False)
    draw.text((W // 2, H + 26), figure_caption, fill=(45, 55, 72), font=font_cap, anchor="mm")
    return out


def make_01_parts():
    """Figure 1.0 - 4 Printed Parts Overview (2x2 clean grid)."""
    panels = [
        ("1. Sourccey Base (cad/stl_sourccey/)", "umi_base", "base_mat", 'pos="0.16 -0.18 0.07" xyaxes="0.74 0.67 0 -0.25 0.28 0.93"'),
        ("2. Sourccey Finger (cad/stl_sourccey/)", "umi_finger", "finger_mat", 'pos="0.15 -0.16 0.08" xyaxes="0.72 0.69 0 -0.28 0.29 0.91"'),
        ("3. Pistol Grip (cad/stl/)", "umi_grip", "grip_mat", 'pos="0.16 -0.30 -0.05" xyaxes="0.84 0.54 0 -0.10 0.16 0.98"'),
        ("4. Trigger Lever (cad/stl/)", "umi_lever", "lever_mat", 'pos="0.12 -0.22 0.01" xyaxes="0.86 0.51 0 -0.10 0.18 0.98"'),
    ]
    pw, ph = 640, 380
    grid = Image.new("RGB", (pw * 2, ph * 2), (255, 255, 255))
    draw = ImageDraw.Draw(grid)
    ft = get_font(20, bold=True)

    for idx, (label, mesh, mat, cam) in enumerate(panels):
        xml = _base_header(pw, ph, fovy=32) + f"""
    <geom type="mesh" mesh="{mesh}" material="{mat}"/>
    <camera name="cam" {cam}/>
  </worldbody></mujoco>"""
        im = _render_tech_view(xml, pw, ph)
        cx, cy = (idx % 2) * pw, (idx // 2) * ph
        grid.paste(im, (cx, cy))
        draw.text((cx + pw // 2, cy + 22), label, fill=(30, 41, 59), font=ft, anchor="mm")

    draw.line([(pw, 16), (pw, ph * 2 - 16)], fill=(226, 232, 240), width=2)
    draw.line([(16, ph), (pw * 2 - 16, ph)], fill=(226, 232, 240), width=2)
    out = _frame_figure(grid, "Figure 1.0 - Four 3D-Printed Parts (PETG, 0.2 mm layers, 4 perimeters, 40% infill)")
    out.save(IMG_DIR / "01_parts.png")


def make_02_inserts():
    """Figure 1.1 - 5x M3 Heat-Set Inserts in the Pistol Grip."""
    inserts = []
    for y, z in HOLES_YZ:
        inserts.append(f'<geom type="cylinder" pos="-0.012 {y:.4f} {z:.4f}" euler="0 1.5708 0" size="0.0024 0.0032" material="brass_mat"/>')
        inserts.append(f'<geom type="capsule" fromto="-0.030 {y:.4f} {z:.4f} -0.015 {y:.4f} {z:.4f}" size="0.0004" material="guide_mat"/>')
    xml = _base_header(1280, 720, fovy=28) + f"""
    <geom type="mesh" mesh="umi_grip" material="grip_mat"/>
    {"".join(inserts)}
    <camera name="cam" pos="0.12 -0.15 0.01" xyaxes="0.72 0.69 0 -0.18 0.19 0.96"/>
  </worldbody></mujoco>"""
    im = _render_tech_view(xml, 1280, 720)
    out = _frame_figure(im, "Figure 1.1 - Pistol Grip Heat-Set Inserts (press flush at ~220 °C)", ["(5) M3 Heat-Set Insert"])
    out.save(IMG_DIR / "02_inserts.png")


def make_03_grip_on_base():
    """Figure 1.2 - Fasten Pistol Grip to Sourccey Base."""
    screws = []
    for y, z in HOLES_YZ:
        screws.append(f'<geom type="cylinder" pos="0.006 {y:.4f} {z:.4f}" euler="0 1.5708 0" size="0.0015 0.010" material="steel_mat"/>')
        screws.append(f'<geom type="cylinder" pos="0.017 {y:.4f} {z:.4f}" euler="0 1.5708 0" size="0.0029 0.0018" material="steel_mat"/>')
        screws.append(f'<geom type="capsule" fromto="-0.072 {y:.4f} {z:.4f} 0.015 {y:.4f} {z:.4f}" size="0.00045" material="guide_mat"/>')
    xml = _base_header(1280, 720, fovy=30) + f"""
    <geom type="mesh" pos="-0.048 0 0" mesh="umi_grip" material="grip_mat"/>
    <geom type="mesh" pos="0 0 0" mesh="umi_base" material="base_mat"/>
    {"".join(screws)}
    <camera name="cam" pos="0.16 -0.32 0.02" xyaxes="0.86 0.51 0 -0.14 0.24 0.96"/>
  </worldbody></mujoco>"""
    im = _render_tech_view(xml, 1280, 720)
    out = _frame_figure(im, "Figure 1.2 - Pistol Grip to Sourccey Base Assembly (install before STS3215 servo)", ["(5) M3x20mm"])
    out.save(IMG_DIR / "03_grip_on_base.png")


def make_04_lever_on_finger():
    """Figure 2.1 - Fasten Trigger Lever to Sourccey Finger (-y side)."""
    screws = []
    for dx, dz in LEVER_HOLES_XZ:
        screws.append(f'<geom type="cylinder" pos="{dx:.4f} -0.095 {dz:.4f}" euler="1.5708 0 0" size="0.0015 0.008" material="steel_mat"/>')
        screws.append(f'<geom type="cylinder" pos="{dx:.4f} -0.104 {dz:.4f}" euler="1.5708 0 0" size="0.0029 0.0018" material="steel_mat"/>')
        screws.append(f'<geom type="capsule" fromto="{dx:.4f} -0.102 {dz:.4f} {dx:.4f} -0.025 {dz:.4f}" size="0.00045" material="guide_mat"/>')
    xml = _base_header(1280, 720, fovy=28) + f"""
    <geom type="mesh" pos="0 0 0" mesh="umi_finger" material="finger_mat"/>
    <geom type="mesh" pos="0 -0.036 0" mesh="umi_lever" material="lever_mat"/>
    {"".join(screws)}
    <camera name="cam" pos="0.16 -0.26 0.05" xyaxes="0.82 0.57 0 -0.14 0.21 0.97"/>
  </worldbody></mujoco>"""
    im = _render_tech_view(xml, 1280, 720)
    out = _frame_figure(im, "Figure 2.1 - Trigger Lever to Sourccey Finger (-y side 4-hole cross)", ["(4) M3x16mm"])
    out.save(IMG_DIR / "04_lever_on_finger.png")


def make_05a_servo_install():
    """Figure 2.2 - Insert Feetech STS3215 Servo into Sourccey Base Cavity."""
    xml = _base_header(1280, 720, fovy=30) + """
    <geom type="mesh" mesh="umi_grip" material="ctx_mat"/>
    <geom type="mesh" mesh="umi_base" material="base_mat"/>
    <!-- STS3215 servo exploded upward/outward cleanly above the base servo bay -->
    <geom type="mesh" pos="0.0 0.0 0.068" mesh="umi_servo" material="servo_mat"/>
    <geom type="capsule" fromto="0.0 -0.005 0.005  0.0 -0.005 0.062" size="0.0007" material="guide_mat"/>
    <geom type="capsule" fromto="-0.020 -0.005 0.005  -0.020 -0.005 0.062" size="0.0007" material="guide_mat"/>
    <camera name="cam" pos="0.18 -0.32 0.08" xyaxes="0.86 0.51 0 -0.18 0.30 0.94"/>
  </worldbody></mujoco>"""
    im = _render_tech_view(xml, 1280, 720)
    out = _frame_figure(
        im,
        "Figure 2.2 - Feetech STS3215 Servo Installation into Sourccey Base Cavity",
        ["(1) Feetech STS3215 7.4V", "Servo Mounting Screws"],
    )
    out.save(IMG_DIR / "05a_servo_install.png")


def make_05_assembled():
    """Figure 2.3 - Mount Finger + Trigger Lever onto STS3215 Horn."""
    xml = _base_header(1280, 720, fovy=30) + """
    <geom type="mesh" mesh="umi_grip" material="ctx_mat"/>
    <geom type="mesh" mesh="umi_base" material="base_mat"/>
    <geom type="mesh" mesh="umi_servo" material="servo_mat"/>
    <body name="hinge" pos="0 0 0">
      <joint name="q" type="hinge" axis="0 -1 0"/>
      <geom type="mesh" mesh="umi_finger" material="finger_mat"/>
      <geom type="mesh" mesh="umi_lever" material="lever_mat"/>
    </body>
    <camera name="cam" pos="0.18 -0.32 0.05" xyaxes="0.85 0.52 0 -0.15 0.25 0.96"/>
  </worldbody></mujoco>"""
    im = _render_tech_view(xml, 1280, 720, qpos=[math.radians(28.0)])
    out = _frame_figure(im, "Figure 2.3 - Finger + Trigger Lever Mounted on STS3215 Horn", ["(1) STS3215 Horn + Screws"])
    out.save(IMG_DIR / "05_assembled_gripper.png")


def make_07b_wiring():
    """Figure 4.2 - 3D Real-CAD Bench Mode (Tethered) Wiring Illustration."""
    xml = _base_header(1280, 720, fovy=28) + """
    <!-- Left: Assembled Sourccey UMI (exact CAD) -->
    <body name="umi" pos="-0.115 0.0 0.035">
      <geom type="mesh" mesh="umi_grip" material="ctx_mat"/>
      <geom type="mesh" mesh="umi_base" material="base_mat"/>
      <geom type="mesh" mesh="umi_servo" material="servo_mat"/>
      <body name="hinge" pos="0 0 0">
        <joint name="q" type="hinge" axis="0 -1 0"/>
        <geom type="mesh" mesh="umi_finger" material="finger_mat"/>
        <geom type="mesh" mesh="umi_lever" material="lever_mat"/>
      </body>
      <body name="cam" pos="-0.016 0.054 -0.0175" euler="0 1.5708 -0.38">
        <geom type="mesh" mesh="cam_cad" material="cam_mat"/>
      </body>
    </body>

    <!-- Center-right: Real Bus Servo Adapter PCB (100k faces) + 7.4V Power Supply (5k faces) -->
    <geom type="mesh" pos="0.060 0.020 -0.055" euler="0.45 0 0" mesh="hat_cad" material="pcb_orange"/>
    <geom type="mesh" pos="0.060 -0.055 -0.095" euler="0.45 0 0" mesh="battery_cad" material="batt_mat"/>

    <!-- 3D Cables: STS3215 3-pin TTL (blue), 7.4V Power (red), Adapter USB (orange), Camera USB (green) -->
    <geom type="capsule" fromto="-0.140 -0.018 0.012  -0.165 -0.035 -0.045" size="0.0022" material="base_mat"/>
    <geom type="capsule" fromto="-0.165 -0.035 -0.045  0.025  0.015 -0.050" size="0.0022" material="base_mat"/>
    <geom type="capsule" fromto=" 0.060 -0.025 -0.075  0.060  0.010 -0.055" size="0.0024" material="spring_mat"/>
    <geom type="capsule" fromto=" 0.092  0.020 -0.052  0.175  0.020 -0.052" size="0.0022" material="pcb_orange"/>
    <geom type="capsule" fromto="-0.132  0.052  0.016 -0.170  0.020 -0.085" size="0.0022" material="lever_mat"/>
    <geom type="capsule" fromto="-0.170  0.020 -0.085  0.175  0.040 -0.085" size="0.0022" material="lever_mat"/>

    <camera name="cam" pos="0.04 -0.44 0.14" xyaxes="1 0 0 0 0.36 0.93"/>
  </worldbody></mujoco>"""
    im = _render_tech_view(xml, 1280, 720, qpos=[math.radians(30.0)])
    d = ImageDraw.Draw(im)
    f = get_font(18, bold=True)
    callouts = [
        ((415, 115), (345, 215), "Sourccey UMI (STS3215 ID 6 + 20x20 Cam)", (37, 99, 235)),
        ((790, 185), (785, 395), "USB Bus Servo Adapter (Waveshare / URT-1)", (215, 95, 15)),
        ((870, 325), (1055, 420), "USB Data -> Host PC (/dev/ttyACM0)", (215, 95, 15)),
        ((870, 445), (1055, 495), "20x20 mm Camera USB -> Host PC (Cam 0)", (22, 130, 65)),
        ((760, 585), (785, 535), "7.4 V DC Supply (required for STS3215 encoder)", (195, 35, 35)),
        ((95, 560), (490, 440), "3-pin TTL Bus Cable", (37, 99, 235)),
    ]
    for (tx, ty), (px, py), label, col in callouts:
        ex = tx + int(d.textlength(label, font=f)) + 8 if tx < 350 else tx - 10
        d.line([(ex, ty + 10), (px, py)], fill=col, width=2)
        d.ellipse([px - 4, py - 4, px + 4, py + 4], fill=col)
        d.text((tx, ty), label, fill=col, font=f)
    out = _frame_figure(
        im,
        "Figure 4.2 - Bench Mode (Tethered) Real-CAD Wiring Setup (STS3215 + 7.4V + 20x20 mm Camera -> PC)",
        ["(1) USB Bus Adapter", "(1) 7.4V Supply", "(2) USB Cables"],
    )
    out.save(IMG_DIR / "07b_wiring.png")


def make_07c_portable_wiring():
    """Figure 4.3 - 3D Real-CAD Portable Wireless Mode (UMI + Pi Zero 2W + 2S LiPo + Adapter + IMU + Button)."""
    xml = _base_header(1280, 720, fovy=28) + """
    <!-- Left: Assembled Sourccey UMI with real IMU + Button + Camera CAD -->
    <body name="umi" pos="-0.130 0.0 0.035">
      <geom type="mesh" mesh="umi_grip" material="ctx_mat"/>
      <geom type="mesh" mesh="umi_base" material="base_mat"/>
      <geom type="mesh" mesh="umi_servo" material="servo_mat"/>
      <geom type="mesh" pos="-0.052 -0.028 -0.008" mesh="imu_cad" material="pcb_purple"/>
      <geom type="mesh" pos="-0.064 -0.037 -0.046" euler="0 -1.5708 0" mesh="button_cad" material="btn_mat"/>
      <body name="hinge" pos="0 0 0">
        <joint name="q" type="hinge" axis="0 -1 0"/>
        <geom type="mesh" mesh="umi_finger" material="finger_mat"/>
        <geom type="mesh" mesh="umi_lever" material="lever_mat"/>
      </body>
      <body name="cam" pos="-0.016 0.054 -0.0175" euler="0 1.5708 -0.38">
        <geom type="mesh" mesh="cam_cad" material="cam_mat"/>
      </body>
    </body>

    <!-- Right: Portable Belt Pack (Real Pi Zero 2W + Real Adapter/5V Board + Real 2S LiPo Battery) -->
    <geom type="mesh" pos="0.085  0.035 -0.015" euler="0.55 0 0" mesh="pi_zero_cad" material="pcb_green"/>
    <geom type="mesh" pos="0.085 -0.015 -0.055" euler="0.55 0 0" mesh="hat_cad" material="pcb_orange"/>
    <geom type="mesh" pos="0.085 -0.075 -0.105" euler="0.55 0 0" mesh="battery_cad" material="batt_mat"/>

    <!-- 3D Interconnect Cables -->
    <!-- Blue: STS3215 3-pin TTL -> Adapter -->
    <geom type="capsule" fromto="-0.165 -0.025 -0.035   0.050 -0.015 -0.052" size="0.0022" material="base_mat"/>
    <!-- Purple: IMU I2C + Button GPIO -> Pi Zero 2W -->
    <geom type="capsule" fromto="-0.180 -0.028  0.025   0.050  0.035 -0.010" size="0.0020" material="pcb_purple"/>
    <!-- Green: 20x20 Camera USB -> Pi OTG Hub -->
    <geom type="capsule" fromto="-0.145  0.050  0.016   0.050  0.042 -0.020" size="0.0020" material="lever_mat"/>
    <!-- Red: 2S LiPo 7.4V -> Adapter & 5V UBEC -> Pi -->
    <geom type="capsule" fromto=" 0.085 -0.045 -0.085   0.085 -0.020 -0.058" size="0.0024" material="spring_mat"/>
    <geom type="capsule" fromto=" 0.095 -0.010 -0.050   0.095  0.025 -0.018" size="0.0022" material="pcb_orange"/>

    <camera name="cam" pos="0.01 -0.46 0.14" xyaxes="1 0 0 0 0.34 0.94"/>
  </worldbody></mujoco>"""
    im = _render_tech_view(xml, 1280, 720, qpos=[math.radians(30.0)])
    d = ImageDraw.Draw(im)
    f = get_font(18, bold=True)
    callouts = [
        ((480, 120), (545, 290), "I2C (IMU) + GPIO 17/27 (Button/LED) + Cam USB -> Pi", (109, 40, 217)),
        ((900, 195), (840, 285), "Raspberry Pi Zero 2W (Wi-Fi)", (22, 130, 65)),
        ((900, 355), (845, 400), "Bus Servo Adapter + 5V 3A UBEC", (215, 95, 15)),
        ((900, 540), (850, 535), "2S LiPo Battery (7.4 V)", (195, 35, 35)),
        ((80, 560), (520, 415), "STS3215 3-pin TTL Bus (7.4 V)", (37, 99, 235)),
    ]
    for (tx, ty), (px, py), label, col in callouts:
        ex = tx + int(d.textlength(label, font=f)) + 8 if tx < 450 else tx - 10
        d.line([(ex, ty + 10), (px, py)], fill=col, width=2)
        d.ellipse([px - 4, py - 4, px + 4, py + 4], fill=col)
        d.text((tx, ty), label, fill=col, font=f)
    d.rounded_rectangle([40, 650, 1240, 698], radius=8, fill=(254, 249, 195), outline=(202, 138, 4), width=2)
    d.text((640, 674), "SAFETY: Never connect 7.4 V LiPo directly to the Raspberry Pi! Always step down to 5.0 V through the UBEC.", fill=(113, 63, 18), font=get_font(16, bold=True), anchor="mm")
    out = _frame_figure(
        im,
        "Figure 4.3 - Portable Wireless Mode Real-CAD Setup (UMI + IMU + Button -> Pi Zero 2W + 2S LiPo Pack)",
        ["Pi Zero 2W + 2S LiPo", "5V 3A UBEC + OTG Hub"],
    )
    out.save(IMG_DIR / "07c_portable_wiring.png")


def make_06b_velcro_strap():
    """Figure 3.2 - Zoomed close-up of threading the 15 mm velcro strap through the trigger pad."""
    xml = _base_header(1280, 720, fovy=24) + """
    <geom type="mesh" mesh="umi_grip" material="ctx_mat"/>
    <geom type="mesh" mesh="umi_base" material="ctx_mat"/>
    <body name="hinge" pos="0 0 0">
      <joint name="q" type="hinge" axis="0 -1 0"/>
      <geom type="mesh" mesh="umi_finger" material="ctx_mat"/>
      <geom type="mesh" mesh="umi_lever" material="lever_mat"/>
      <!-- Black 15mm velcro strap threaded cleanly through the two side slots (-y=-0.042, -0.030) -->
      <geom type="capsule" fromto="-0.017 -0.042 -0.042  -0.002 -0.042 -0.042" size="0.0026" material="cam_mat"/>
      <geom type="capsule" fromto="-0.017 -0.030 -0.042  -0.002 -0.030 -0.042" size="0.0026" material="cam_mat"/>
      <geom type="capsule" fromto="-0.002 -0.042 -0.042   0.016 -0.042 -0.042" size="0.0026" material="cam_mat"/>
      <geom type="capsule" fromto="-0.002 -0.030 -0.042   0.016 -0.030 -0.042" size="0.0026" material="cam_mat"/>
      <geom type="capsule" fromto=" 0.016 -0.042 -0.042   0.022 -0.036 -0.042" size="0.0026" material="cam_mat"/>
      <geom type="capsule" fromto=" 0.016 -0.030 -0.042   0.022 -0.036 -0.042" size="0.0026" material="cam_mat"/>
      <geom type="capsule" fromto="-0.017 -0.042 -0.042  -0.017 -0.030 -0.042" size="0.0026" material="cam_mat"/>
    </body>
    <camera name="cam" pos="0.11 -0.18 -0.03" xyaxes="0.82 0.57 0 -0.06 0.08 0.99"/>
  </worldbody></mujoco>"""
    im = _render_tech_view(xml, 1280, 720, qpos=[math.radians(30.0)])
    d = ImageDraw.Draw(im)
    f = get_font(18, bold=True)
    d.text((870, 390), "Loop around index finger", fill=(20, 28, 40), font=f)
    d.line([(860, 398), (745, 420)], fill=(20, 28, 40), width=2)
    d.text((870, 480), "Thread through both side slots", fill=(22, 130, 72), font=f)
    d.line([(860, 488), (690, 468)], fill=(22, 130, 72), width=2)
    out = _frame_figure(
        im,
        "Figure 3.2 - Threading the 15 mm Velcro Finger Strap through the Trigger Pad Slots",
        ["(1) 15mm Velcro Strap (~200mm)"],
    )
    out.save(IMG_DIR / "06b_velcro_strap.png")


def make_07d_cable_routing():
    """Figure 4.1 - 3D view of Cable Routing, BNO085 IMU, Pushbutton, LED & Zip Ties on the Grip."""
    xml = _base_header(1280, 720, fovy=30) + """
    <geom type="mesh" mesh="umi_grip" material="grip_mat"/>
    <geom type="mesh" mesh="umi_base" material="ctx_mat"/>
    <geom type="mesh" mesh="umi_servo" material="servo_mat"/>
    <body name="hinge" pos="0 0 0">
      <joint name="q" type="hinge" axis="0 -1 0"/>
      <geom type="mesh" mesh="umi_finger" material="ctx_mat"/>
      <geom type="mesh" mesh="umi_lever" material="ctx_mat"/>
    </body>

    <!-- Real I2C IMU Breakout CAD (7.8k faces) mounted flat on top of the grip beam -->
    <geom type="mesh" pos="-0.052 -0.028 -0.008" mesh="imu_cad" material="pcb_purple"/>

    <!-- Real Tactile Start/Stop Pushbutton CAD on upper rear of pistol grip + Status LED -->
    <geom type="mesh" pos="-0.064 -0.037 -0.046" euler="0 -1.5708 0" mesh="button_cad" material="btn_mat"/>
    <geom type="sphere" pos="-0.065 -0.037 -0.060" size="0.0032" material="spring_mat"/>

    <!-- 3-pin STS3215 Servo Cable (blue) routing out of base and down rear of pistol grip -->
    <geom type="capsule" fromto="-0.025 -0.015 -0.022  -0.068 -0.030 -0.030" size="0.0022" material="base_mat"/>
    <geom type="capsule" fromto="-0.068 -0.030 -0.030  -0.068 -0.032 -0.148" size="0.0022" material="base_mat"/>

    <!-- 20x20 mm Wrist Camera USB Cable (green) routing from +y wing around back down rear of grip -->
    <geom type="capsule" fromto="-0.018 0.052 -0.018   -0.068 0.010 -0.025" size="0.0022" material="lever_mat"/>
    <geom type="capsule" fromto="-0.068 0.010 -0.025   -0.068 -0.038 -0.034" size="0.0022" material="lever_mat"/>
    <geom type="capsule" fromto="-0.068 -0.038 -0.034  -0.068 -0.040 -0.148" size="0.0022" material="lever_mat"/>

    <!-- Zip ties securing cables at z = -0.075 and z = -0.118 -->
    <geom type="box" pos="-0.046 -0.037 -0.075" size="0.023 0.017 0.002" rgba="0.10 0.10 0.12 1"/>
    <geom type="box" pos="-0.046 -0.037 -0.118" size="0.023 0.017 0.002" rgba="0.10 0.10 0.12 1"/>

    <!-- Rear-3/4 view showing back of pistol grip, IMU, button, LED, cables & zip ties -->
    <camera name="cam" pos="-0.26 -0.24 0.06" xyaxes="0.68 -0.73 0 0.22 0.20 0.95"/>
  </worldbody></mujoco>"""
    im = _render_tech_view(xml, 1280, 720, qpos=[math.radians(30.0)])
    d = ImageDraw.Draw(im)
    f = get_font(18, bold=True)
    # Clean non-overlapping callouts on right & left margins
    callouts = [
        ((820, 210), (655, 290), "BNO085 / MPU6050 IMU (flat on beam)", (109, 40, 217)),
        ((820, 290), (618, 365), "Record Button (GPIO 17) + Status LED (GPIO 27)", (180, 115, 0)),
        ((820, 385), (630, 445), "STS3215 Cable (blue) + Camera USB (green)", (29, 78, 216)),
        ((820, 475), (620, 515), "Zip Ties (leave gentle slack at wrist)", (30, 41, 59)),
    ]
    for (tx, ty), (px, py), label, col in callouts:
        d.line([(tx - 10, ty + 10), (px, py)], fill=col, width=2)
        d.ellipse([px - 4, py - 4, px + 4, py + 4], fill=col)
        d.text((tx, ty), label, fill=col, font=f)
    out = _frame_figure(
        im,
        "Figure 4.1 - Cable Routing, Zip Ties, Optional IMU & Grip Pushbutton Placement",
        ["(4) Zip Ties", "Optional: IMU + Button + LED"],
    )
    out.save(IMG_DIR / "07d_cable_routing.png")


def make_07e_pi_gpio_pinout():
    """Figure 4.4 - Exact Raspberry Pi Zero 2W 40-Pin GPIO Pinout for Portable Mode."""
    W, H = 1280, 640
    img = Image.new("RGB", (W, H), (255, 255, 255))
    draw = ImageDraw.Draw(img)
    ft = get_font(20, bold=True)
    fs = get_font(15, bold=True)
    fn = get_font(14, bold=False)

    draw.text((W // 2, 34), "Raspberry Pi Zero 2W  -  GPIO Header Connections (Top 16 Pins)", fill=(20, 28, 40), font=ft, anchor="mm")

    # Draw pins 1..16 (2 columns x 8 rows) cleanly centered
    cx_l, cx_r = 560, 720
    y_start, dy = 98, 58
    draw.rounded_rectangle([cx_l - 38, y_start - 30, cx_r + 38, y_start + 7 * dy + 30], radius=12, fill=(241, 245, 249), outline=(71, 85, 105), width=2)

    pins = [
        # (num, col_side, row, color, name, dest)
        (1,  "L", 0, (234, 88, 12),   "3.3V Power",      "-> IMU VCC (3.3V)"),
        (2,  "R", 0, (220, 38, 38),   "5V Power",        "<- 5V 3A UBEC (+5V)"),
        (3,  "L", 1, (124, 58, 237),  "GPIO 2 (SDA1)",   "-> IMU SDA"),
        (4,  "R", 1, (220, 38, 38),   "5V Power",        "(spare 5V)"),
        (5,  "L", 2, (124, 58, 237),  "GPIO 3 (SCL1)",   "-> IMU SCL"),
        (6,  "R", 2, (30, 41, 59),    "GND",             "<- UBEC GND + IMU GND"),
        (7,  "L", 3, (148, 163, 184), "GPIO 4",          "(unused)"),
        (8,  "R", 3, (148, 163, 184), "GPIO 14 (TX)",    "(unused)"),
        (9,  "L", 4, (30, 41, 59),    "GND",             "<- Button GND + LED/Buzzer GND"),
        (10, "R", 4, (148, 163, 184), "GPIO 15 (RX)",    "(unused)"),
        (11, "L", 5, (202, 138, 4),   "GPIO 17",         "-> Start/Stop Pushbutton"),
        (12, "R", 5, (148, 163, 184), "GPIO 18",         "(unused)"),
        (13, "L", 6, (22, 163, 74),   "GPIO 27",         "-> Status LED (+330 ohm) / Buzzer"),
        (14, "R", 6, (30, 41, 59),    "GND",             "(common GND)"),
        (15, "L", 7, (148, 163, 184), "GPIO 22",         "(unused)"),
        (16, "R", 7, (148, 163, 184), "GPIO 23",         "(unused)"),
    ]
    for num, side, row, col, name, dest in pins:
        px = cx_l if side == "L" else cx_r
        py = y_start + row * dy
        draw.ellipse([px - 17, py - 17, px + 17, py + 17], fill=col, outline=(255, 255, 255), width=2)
        draw.text((px, py), str(num), fill=(255, 255, 255), font=fs, anchor="mm")
        active = dest != "(unused)"
        tc = (20, 28, 40) if active else (148, 163, 184)
        if side == "L":
            draw.text((px - 52, py - 8), name, fill=tc, font=fs if active else fn, anchor="rm")
            if active:
                draw.text((px - 52, py + 11), dest, fill=col, font=fs, anchor="rm")
        else:
            draw.text((px + 52, py - 8), name, fill=tc, font=fs if active else fn, anchor="lm")
            if active:
                draw.text((px + 52, py + 11), dest, fill=col, font=fs, anchor="lm")

    draw.rounded_rectangle([60, 560, W - 60, 610], radius=8, fill=(239, 246, 255), outline=(37, 99, 235), width=2)
    draw.text((W // 2, 585), "Enable I2C on the Pi once:  sudo raspi-config -> Interface Options -> I2C -> Enable", fill=(30, 58, 138), font=fs, anchor="mm")

    out = _frame_figure(img, "Figure 4.4 - Raspberry Pi Zero 2W GPIO Header Pinout (IMU, Button, LED/Buzzer, 5V UBEC)")
    out.save(IMG_DIR / "07e_pi_gpio_pinout.png")


def make_07f_portable_pack_3d():
    """Figure 4.5 - 100% Real 3D CAD Layout of the Portable Electronics Pack (Pi Zero 2W, LiPo, Adapter HAT, IMU)."""
    xml = _base_header(1280, 720, fovy=28) + """
    <!-- Real 5k-face LiPo Battery Pack CAD (left) -->
    <geom type="mesh" pos="-0.048 0.0 0.0" mesh="battery_cad" material="batt_mat"/>

    <!-- Real 103k-face Raspberry Pi Zero 2W CAD (right-top) -->
    <geom type="mesh" pos="0.044 0.024 0.0" mesh="pi_zero_cad" material="pcb_green"/>

    <!-- Real 100k-face Bus / Power Adapter Board CAD (right-bottom) -->
    <geom type="mesh" pos="0.044 -0.024 0.0" mesh="hat_cad" material="pcb_orange"/>

    <!-- Real 7.8k-face IMU Breakout CAD + Real Pushbutton CAD (front-center) -->
    <geom type="mesh" pos="-0.005 -0.048 0.0" mesh="imu_cad" material="pcb_purple"/>
    <geom type="mesh" pos="-0.038 -0.048 0.0" mesh="button_cad" material="btn_mat"/>

    <!-- Clean power/data interconnect wires -->
    <geom type="capsule" fromto="-0.010 0.005 0.008   0.012 -0.020 0.008" size="0.0014" material="spring_mat"/>
    <geom type="capsule" fromto=" 0.012 -0.010 0.008   0.012  0.018 0.008" size="0.0014" material="base_mat"/>
    <geom type="capsule" fromto="-0.005 -0.036 0.004   0.022  0.014 0.004" size="0.0012" material="pcb_purple"/>

    <camera name="cam" pos="0.0 -0.20 0.19" xyaxes="1 0 0 0 0.69 0.72"/>
  </worldbody></mujoco>"""
    im = _render_tech_view(xml, 1280, 720)
    d = ImageDraw.Draw(im)
    f = get_font(18, bold=True)
    labels = [
        ((70, 175), (430, 290), "2S LiPo Battery Pack (7.4 V)", (195, 35, 35)),
        ((70, 505), (490, 495), "Grip Pushbutton (GPIO 17)", (175, 115, 0)),
        ((70, 585), (615, 515), "BNO085 / MPU6050 IMU (I2C)", (109, 40, 217)),
        ((915, 150), (800, 240), "Raspberry Pi Zero 2W", (22, 130, 65)),
        ((915, 430), (815, 415), "USB Bus Servo / 5V Power Board", (215, 95, 15)),
    ]
    for (tx, ty), (px, py), text, col in labels:
        ex = tx + int(d.textlength(text, font=f)) + 8 if tx < 500 else tx - 10
        d.line([(ex, ty + 10), (px, py)], fill=col, width=2)
        d.ellipse([px - 4, py - 4, px + 4, py + 4], fill=col)
        d.text((tx, ty), text, fill=col, font=f)
    out = _frame_figure(im, "Figure 4.5 - Portable Electronics Pack Real 3D CAD Components (Pi Zero 2W, Battery, Adapter, IMU, Button)")
    out.save(IMG_DIR / "07f_portable_pack_3d.png")


def make_07g_bimanual_wiring():
    """Figure 4.6 - 3D Real-CAD Bimanual (Dual-Arm) Daisy-Chain Setup."""
    xml = _base_header(1280, 720, fovy=30) + """
    <!-- LEFT Sourccey UMI (ID 6) -->
    <body name="umi_left" pos="-0.155 0.035 0.040">
      <geom type="mesh" mesh="umi_grip" material="ctx_mat"/>
      <geom type="mesh" mesh="umi_base" material="base_mat"/>
      <geom type="mesh" mesh="umi_servo" material="servo_mat"/>
      <body name="hinge_l" pos="0 0 0">
        <joint name="ql" type="hinge" axis="0 -1 0"/>
        <geom type="mesh" mesh="umi_finger" material="finger_mat"/>
        <geom type="mesh" mesh="umi_lever" material="lever_mat"/>
      </body>
      <body name="cam_l" pos="-0.016 0.054 -0.0175" euler="0 1.5708 -0.38">
        <geom type="mesh" mesh="cam_cad" material="cam_mat"/>
      </body>
    </body>

    <!-- RIGHT Sourccey UMI (ID 16) -->
    <body name="umi_right" pos="0.015 0.035 0.040">
      <geom type="mesh" mesh="umi_grip" material="ctx_mat"/>
      <geom type="mesh" mesh="umi_base" material="base_mat"/>
      <geom type="mesh" mesh="umi_servo" material="servo_mat"/>
      <body name="hinge_r" pos="0 0 0">
        <joint name="qr" type="hinge" axis="0 -1 0"/>
        <geom type="mesh" mesh="umi_finger" material="finger_mat"/>
        <geom type="mesh" mesh="umi_lever" material="lever_mat"/>
      </body>
      <body name="cam_r" pos="-0.016 0.054 -0.0175" euler="0 1.5708 -0.38">
        <geom type="mesh" mesh="cam_cad" material="cam_mat"/>
      </body>
    </body>

    <!-- Single Shared Bus Servo Adapter + 7.4V Supply (right) -->
    <geom type="mesh" pos="0.165 0.015 -0.045" euler="0.50 0 0" mesh="hat_cad" material="pcb_orange"/>
    <geom type="mesh" pos="0.165 -0.055 -0.095" euler="0.50 0 0" mesh="battery_cad" material="batt_mat"/>

    <!-- Daisy-chain 3-pin TTL bus: Left (ID 6) -> Right (ID 16) -> Single Adapter -->
    <geom type="capsule" fromto="-0.195 0.010 -0.045  -0.020 0.010 -0.045" size="0.0024" material="base_mat"/>
    <geom type="capsule" fromto="-0.020 0.010 -0.045   0.130 0.010 -0.045" size="0.0024" material="base_mat"/>
    <!-- 7.4V Power -->
    <geom type="capsule" fromto=" 0.165 -0.025 -0.075  0.165 0.005 -0.048" size="0.0024" material="spring_mat"/>
    <!-- Both Camera USB cables -->
    <geom type="capsule" fromto="-0.170 0.085  0.020  -0.170 0.085 -0.095" size="0.0020" material="lever_mat"/>
    <geom type="capsule" fromto=" 0.000 0.085  0.020   0.000 0.085 -0.095" size="0.0020" material="lever_mat"/>

    <camera name="cam" pos="0.01 -0.52 0.15" xyaxes="1 0 0 0 0.34 0.94"/>
  </worldbody></mujoco>"""
    im = _render_tech_view(xml, 1280, 720, qpos=[math.radians(32.0), math.radians(24.0)])
    d = ImageDraw.Draw(im)
    f = get_font(18, bold=True)
    callouts = [
        ((70, 120), (315, 250), "LEFT UMI (STS3215 ID 6 + Cam 0)", (37, 99, 235)),
        ((470, 120), (650, 250), "RIGHT UMI (STS3215 ID 16 + Cam 2)", (22, 130, 65)),
        ((870, 225), (965, 425), "SINGLE Bus Adapter (ID 6 + ID 16)", (215, 95, 15)),
        ((870, 565), (965, 550), "7.4 V Power Supply", (195, 35, 35)),
        ((95, 575), (465, 445), "3-pin Daisy-Chain Bus Cable (ID 6 <-> ID 16)", (37, 99, 235)),
    ]
    for (tx, ty), (px, py), label, col in callouts:
        ex = tx + int(d.textlength(label, font=f)) + 8 if tx < 450 else tx - 10
        d.line([(ex, ty + 10), (px, py)], fill=col, width=2)
        d.ellipse([px - 4, py - 4, px + 4, py + 4], fill=col)
        d.text((tx, ty), label, fill=col, font=f)
    out = _frame_figure(
        im,
        "Figure 4.6 - Bimanual (Dual-Arm) Daisy-Chain Setup Sharing One Bus Servo Adapter",
        ["LEFT: ID 6 + Cam 0", "RIGHT: ID 16 + Cam 2"],
    )
    out.save(IMG_DIR / "07g_bimanual_wiring.png")


def make_08b_aruco_placement():
    """Figure 5.0 - ArUco Marker Placement: ID 0 (finger), ID 1 (jaw), ID 10 (80mm tabletop anchor)."""
    xml = _base_header(1280, 720, fovy=32) + """
    <!-- Tabletop plane -->
    <geom type="box" pos="0.04 0 -0.140" size="0.22 0.18 0.003" rgba="0.93 0.94 0.96 1"/>
    <!-- Printed 80mm Tabletop ArUco ID 10 Anchor -->
    <geom type="box" pos="0.11 0.01 -0.136" size="0.042 0.042 0.0008" rgba="1 1 1 1"/>
    <geom type="box" pos="0.11 0.01 -0.135" size="0.036 0.036 0.0008" rgba="0.10 0.10 0.12 1"/>
    <geom type="box" pos="0.10 0.02 -0.134" size="0.011 0.011 0.0008" rgba="1 1 1 1"/>
    <geom type="box" pos="0.12 -0.00 -0.134" size="0.011 0.011 0.0008" rgba="1 1 1 1"/>

    <!-- UMI hovering cleanly above tabletop -->
    <body name="umi" pos="-0.02 0 -0.005">
      <geom type="mesh" mesh="umi_grip" material="ctx_mat"/>
      <geom type="mesh" mesh="umi_base" material="base_mat"/>
      <!-- ArUco ID 1 (20mm) on outer blue fixed jaw -->
      <geom type="box" pos="0.032 -0.022 -0.036" size="0.010 0.001 0.010" rgba="1 1 1 1"/>
      <geom type="box" pos="0.032 -0.023 -0.036" size="0.007 0.001 0.007" rgba="0.10 0.10 0.12 1"/>
      <body name="hinge" pos="0 0 0">
        <joint name="q" type="hinge" axis="0 -1 0"/>
        <geom type="mesh" mesh="umi_finger" material="finger_mat"/>
        <geom type="mesh" mesh="umi_lever" material="lever_mat"/>
        <!-- ArUco ID 0 (20mm) on top flat face of orange finger -->
        <geom type="box" pos="0.044 0.0 0.016" euler="0 0.25 0" size="0.010 0.010 0.001" rgba="1 1 1 1"/>
        <geom type="box" pos="0.044 0.0 0.017" euler="0 0.25 0" size="0.007 0.007 0.001" rgba="0.10 0.10 0.12 1"/>
      </body>
    </body>

    <camera name="cam" pos="0.24 -0.38 0.12" xyaxes="0.85 0.52 0 -0.22 0.36 0.91"/>
  </worldbody></mujoco>"""
    im = _render_tech_view(xml, 1280, 720, qpos=[math.radians(32.0)])
    d = ImageDraw.Draw(im)
    f = get_font(18, bold=True)
    callouts = [
        ((790, 155), (610, 210), "ArUco ID 0 (20x20 mm) on orange finger", (215, 95, 15)),
        ((790, 265), (595, 360), "ArUco ID 1 (20x20 mm) on blue fixed jaw", (37, 99, 235)),
        ((875, 480), (775, 515), "ArUco ID 10 (80x80 mm) on table", (20, 28, 40)),
    ]
    for (tx, ty), (px, py), label, col in callouts:
        d.line([(tx - 10, ty + 10), (px, py)], fill=col, width=2)
        d.ellipse([px - 4, py - 4, px + 4, py + 4], fill=col)
        d.text((tx, ty), label, fill=col, font=f)
    out = _frame_figure(
        im,
        "Figure 5.0 - ArUco Sticker Placement (On-Grip Clock Sync + Tabletop Metric Scale Anchor)",
        ["(2) 20mm ArUco (ID 0, ID 1)", "(1) 80mm Tabletop ArUco (ID 10)"],
    )
    out.save(IMG_DIR / "08b_aruco_placement.png")


def make_06_spring():
    """Figure 3.1 - Anchor Pin & Extension Spring (Closed vs Open side view)."""
    pw, ph = 640, 560
    panels = []
    for q_deg, title in [(-5.0, "Closed (q = -5°)  ·  Spring = 59 mm"), (60.0, "Open Stop (q = 60°)  ·  Spring = 28 mm")]:
        q_rad = math.radians(q_deg)
        c, s = math.cos(-q_rad), math.sin(-q_rad)
        tx, tz = -0.016 * c + 0.024 * s, 0.016 * s + 0.024 * c
        xml = _base_header(pw, ph, fovy=30) + f"""
    <geom type="mesh" mesh="umi_grip" material="ctx_mat"/>
    <geom type="mesh" mesh="umi_base" material="ctx_mat"/>
    <geom type="cylinder" pos="-0.042 -0.020 -0.026" euler="1.5708 0 0" size="0.0022 0.036" material="steel_mat"/>
    <geom type="capsule" fromto="-0.042 -0.036 -0.026 {tx:.5f} -0.036 {tz:.5f}" size="0.0026" material="spring_mat"/>
    <body name="hinge" pos="0 0 0">
      <joint name="q" type="hinge" axis="0 -1 0"/>
      <geom type="mesh" mesh="umi_finger" material="finger_mat"/>
      <geom type="mesh" mesh="umi_lever" material="lever_mat"/>
    </body>
    <camera name="cam" pos="-0.01 -0.28 -0.01" xyaxes="1 0 0 0 0 1"/>
  </worldbody></mujoco>"""
        im = _render_tech_view(xml, pw, ph, qpos=[q_rad])
        d = ImageDraw.Draw(im)
        d.text((pw // 2, ph - 22), title, fill=(30, 41, 59), font=get_font(19, bold=True), anchor="mm")
        panels.append(im)

    comb = Image.new("RGB", (pw * 2, ph), (255, 255, 255))
    comb.paste(panels[0], (0, 0))
    comb.paste(panels[1], (pw, 0))
    ImageDraw.Draw(comb).line([(pw, 20), (pw, ph - 20)], fill=(226, 232, 240), width=2)
    out = _frame_figure(
        comb,
        "Figure 3.1 - M3x70 Anchor Pin & Extension Spring (red) pulling jaws open",
        ["(1) M3x70mm + M3 Nut", "(1) Extension Spring", "(1) 15mm Velcro Strap"],
    )
    out.save(IMG_DIR / "06_spring_and_pin.png")


def make_07_camera():
    """Figure 3.3 - Real 20x20 mm Wrist Camera CAD Mesh in +y Side Wing."""
    xml = _base_header(1280, 720, fovy=30) + """
    <geom type="mesh" mesh="umi_grip" material="ctx_mat"/>
    <geom type="mesh" mesh="umi_base" material="base_mat"/>
    <body name="hinge" pos="0 0 0">
      <joint name="q" type="hinge" axis="0 -1 0"/>
      <geom type="mesh" mesh="umi_finger" material="finger_mat"/>
    </body>
    <!-- Real camera CAD mesh floating behind the +y Ø25 mm wing opening -->
    <body name="cam_mod" pos="-0.036 0.095 -0.0175" euler="0 1.5708 -0.38">
      <geom type="mesh" mesh="cam_cad" material="cam_mat"/>
    </body>
    <geom type="capsule" fromto="-0.036 0.095 -0.0175  -0.011 0.050 -0.0175" size="0.0006" material="guide_mat"/>
    <camera name="cam" pos="-0.14 0.25 0.14" xyaxes="-0.82 -0.57 0 0.28 -0.40 0.87"/>
  </worldbody></mujoco>"""
    im = _render_tech_view(xml, 1280, 720, qpos=[math.radians(30.0)])
    out = _frame_figure(
        im,
        "Figure 3.3 - Sourccey 20x20 mm Wrist Camera mounted behind the +y Ø25 mm wing opening",
        ["(1) 20x20mm Wrist Camera"],
    )
    out.save(IMG_DIR / "07_camera.png")


def make_09_operation():
    """Figure 4.1 - Trigger Operation (Open 60°, Mid 30°, Closed -5°)."""
    pw, ph = 460, 480
    panels = []
    for q_deg, title in [(60.0, "Released: Open (q = 60°)"), (30.0, "Halfway (q = 30°)"), (-5.0, "Pulled: Closed (q = -5°)")]:
        xml = _base_header(pw, ph, fovy=32) + """
    <geom type="mesh" mesh="umi_grip" material="grip_mat"/>
    <geom type="mesh" mesh="umi_base" material="base_mat"/>
    <body name="hinge" pos="0 0 0">
      <joint name="q" type="hinge" axis="0 -1 0"/>
      <geom type="mesh" mesh="umi_finger" material="finger_mat"/>
      <geom type="mesh" mesh="umi_lever" material="lever_mat"/>
    </body>
    <camera name="cam" pos="0.16 -0.34 0.01" xyaxes="0.88 0.47 0 -0.10 0.19 0.98"/>
  </worldbody></mujoco>"""
        im = _render_tech_view(xml, pw, ph, qpos=[math.radians(q_deg)])
        ImageDraw.Draw(im).text((pw // 2, ph - 20), title, fill=(30, 41, 59), font=get_font(18, bold=True), anchor="mm")
        panels.append(im)

    comb = Image.new("RGB", (pw * 3, ph), (255, 255, 255))
    for i, im in enumerate(panels):
        comb.paste(im, (i * pw, 0))
    out = _frame_figure(comb, "Figure 4.1 - Trigger Sweep & STS3215 Gripper Joint Range (-5° to 60°)")
    out.save(IMG_DIR / "09_operation.png")


if __name__ == "__main__":
    ensure_assets()
    ensure_electronics_cad()
    make_01_parts()
    make_02_inserts()
    make_03_grip_on_base()
    make_04_lever_on_finger()
    make_05a_servo_install()
    make_05_assembled()
    make_06_spring()
    make_06b_velcro_strap()
    make_07_camera()
    make_07d_cable_routing()
    make_07b_wiring()
    make_07c_portable_wiring()
    make_07e_pi_gpio_pinout()
    make_07f_portable_pack_3d()
    make_07g_bimanual_wiring()
    make_08b_aruco_placement()
    make_09_operation()
    print("Generated all Sourccey-manual style assembly figures in", IMG_DIR)
