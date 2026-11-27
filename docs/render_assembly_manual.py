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
    <material name="cam_mat" rgba="0.14 0.16 0.20 1" roughness="0.35" specular="0.35"/>
    <material name="lens_mat" rgba="0.06 0.12 0.28 1" roughness="0.10" specular="0.90"/>
    <material name="guide_mat" rgba="0.35 0.40 0.48 0.9"/>

    <mesh name="umi_base" file="umi_base.stl"/>
    <mesh name="umi_finger" file="umi_finger.stl"/>
    <mesh name="umi_servo" file="umi_servo.stl"/>
    <mesh name="umi_grip" file="umi_grip.stl"/>
    <mesh name="umi_lever" file="umi_lever.stl"/>
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
    """Figure 4.1 - Clean Sourccey-manual style Electronics & Wiring Diagram."""
    W, H = 1280, 680
    img = Image.new("RGB", (W, H), (255, 255, 255))
    draw = ImageDraw.Draw(img)

    f_title = get_font(20, bold=True)
    f_sub = get_font(15, bold=False)
    f_wire = get_font(15, bold=True)
    f_warn = get_font(16, bold=True)

    def box(x0, y0, x1, y1, fill, outline, title, lines):
        draw.rounded_rectangle([x0, y0, x1, y1], radius=10, fill=fill, outline=outline, width=3)
        draw.text(((x0 + x1) // 2, y0 + 24), title, fill=(20, 28, 40), font=f_title, anchor="mm")
        for i, ln in enumerate(lines):
            draw.text(((x0 + x1) // 2, y0 + 54 + i * 22), ln, fill=(65, 78, 96), font=f_sub, anchor="mm")

    # Left: Sourccey UMI Hardware Group
    draw.rounded_rectangle([38, 92, 412, 540], radius=14, fill=(248, 250, 252), outline=(148, 163, 184), width=2)
    draw.text((225, 118), "SOURCCEY UMI (HANDHELD)", fill=(51, 65, 85), font=f_wire, anchor="mm")

    box(66, 148, 384, 290, (239, 246, 255), (37, 99, 235), "Feetech STS3215 Servo", [
        "7.4 V bus servo (ID 6)",
        "Torque OFF (read-only encoder)",
        "0.088 deg resolution @ ~200 Hz",
    ])
    box(66, 355, 384, 495, (240, 253, 244), (22, 163, 74), "Sourccey 20x20 mm Camera", [
        "Mounted behind +y Ø25 mm wing",
        "Standard USB UVC video stream",
        "Same viewpoint as Sourccey arm",
    ])

    # Middle: Bus Servo Adapter + 7.4V PSU
    box(495, 148, 825, 290, (255, 247, 237), (234, 88, 12), "USB Bus Servo Adapter", [
        "Waveshare Bus Servo Adapter",
        "or Feetech URT-1 (1 Mbps TTL)",
        "Bridges 3-pin bus <-> USB",
    ])
    box(520, 355, 800, 475, (254, 242, 242), (220, 38, 38), "7.4 V DC Power Supply", [
        "2S LiPo or 7.4 V bench adapter",
        "Powers STS3215 encoder",
        "Check + / - polarity!",
    ])

    # Right: Host PC / Raspberry Pi
    box(920, 210, 1235, 430, (241, 245, 249), (30, 41, 59), "PC / Raspberry Pi", [
        "Runs record_episode.py or",
        "Sourccey UMI Studio (space/app.py)",
        "Serial: /dev/ttyACM0 (or COMx)",
        "Video: USB Camera 0",
        "Shared monotonic clock sync",
    ])

    # Wires
    # 1. STS3215 <-> Adapter (3-pin TTL bus)
    draw.line([(384, 218), (495, 218)], fill=(37, 99, 235), width=5)
    draw.text((440, 198), "3-pin Bus", fill=(37, 99, 235), font=f_wire, anchor="mm")
    draw.text((440, 236), "(TTL)", fill=(71, 85, 105), font=f_sub, anchor="mm")

    # 2. 7.4V PSU -> Adapter (Red/Black power)
    draw.line([(652, 355), (652, 290)], fill=(220, 38, 38), width=5)
    draw.line([(668, 355), (668, 290)], fill=(30, 41, 59), width=5)
    draw.text((748, 322), "7.4 V (+ / GND)", fill=(220, 38, 38), font=f_wire, anchor="mm")

    # 3. Adapter -> PC (USB-C)
    draw.line([(825, 218), (875, 218), (875, 285), (920, 285)], fill=(234, 88, 12), width=5)
    draw.text((872, 200), "USB", fill=(234, 88, 12), font=f_wire, anchor="mm")

    # 4. Wrist Camera -> PC (USB direct, routed cleanly along bottom)
    draw.line([(384, 495), (384, 520), (875, 520), (875, 375), (920, 375)], fill=(22, 163, 74), width=5)
    draw.text((640, 504), "USB Cable (route with zip ties down rear of pistol grip)", fill=(22, 163, 74), font=f_wire, anchor="mm")

    # Safety callout banner at bottom
    draw.rounded_rectangle([38, 575, 1235, 635], radius=8, fill=(254, 249, 195), outline=(202, 138, 4), width=2)
    draw.text((636, 605), "IMPORTANT: Power STS3215 with 7.4 V via the bus adapter (USB 5 V alone will not power the encoder). Keep torque OFF.", fill=(113, 63, 18), font=f_warn, anchor="mm")

    out = _frame_figure(
        img,
        "Figure 4.1 - Sourccey UMI Electronics & Wiring Diagram (STS3215 + 20x20 mm Wrist Camera)",
    )
    out.save(IMG_DIR / "07b_wiring.png")


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
    """Figure 3.2 - 20x20 mm Sourccey Wrist Camera in +y Side Wing."""
    xml = _base_header(1280, 720, fovy=30) + """
    <geom type="mesh" mesh="umi_grip" material="ctx_mat"/>
    <geom type="mesh" mesh="umi_base" material="base_mat"/>
    <body name="hinge" pos="0 0 0">
      <joint name="q" type="hinge" axis="0 -1 0"/>
      <geom type="mesh" mesh="umi_finger" material="finger_mat"/>
    </body>
    <!-- 20x20 mm camera floating slightly behind the +y Ø25 mm wing opening -->
    <body name="cam_mod" pos="-0.036 0.095 -0.0175" euler="0 0 -0.38">
      <geom type="box" pos="-0.004 0 0" size="0.0035 0.010 0.010" material="cam_mat"/>
      <geom type="cylinder" pos="0.003 0 0" euler="0 1.5708 0" size="0.0068 0.0045" material="cam_mat"/>
      <geom type="cylinder" pos="0.0078 0 0" euler="0 1.5708 0" size="0.0048 0.0008" material="lens_mat"/>
    </body>
    <geom type="capsule" fromto="-0.036 0.095 -0.0175  -0.011 0.050 -0.0175" size="0.0006" material="guide_mat"/>
    <!-- View from +y rear-top looking cleanly at the Ø25 mm wing opening -->
    <camera name="cam" pos="-0.14 0.25 0.14" xyaxes="-0.82 -0.57 0 0.28 -0.40 0.87"/>
  </worldbody></mujoco>"""
    im = _render_tech_view(xml, 1280, 720, qpos=[math.radians(30.0)])
    out = _frame_figure(
        im,
        "Figure 3.2 - Sourccey 20x20 mm Wrist Camera mounted behind the +y Ø25 mm wing opening",
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
    make_01_parts()
    make_02_inserts()
    make_03_grip_on_base()
    make_04_lever_on_finger()
    make_05a_servo_install()
    make_05_assembled()
    make_06_spring()
    make_07_camera()
    make_07b_wiring()
    make_09_operation()
    print("Generated all Sourccey-manual style assembly figures in", IMG_DIR)
