"""Render 100% CAD-exact studio views (Hero + Exploded Assembly) in MuJoCo.

Guarantees identical geometry and exact camera placement (ONLY the 20x20 mm
Sourccey wrist camera behind the Ø25 mm opening on the blue base's +y wing)
across all repository visuals.

Generates:
  - docs/img/00_hero.png
  - docs/img/00_exploded_overview.png
"""
import math
import pathlib
import sys

import mujoco
import numpy as np
import trimesh
from PIL import Image, ImageDraw, ImageFont
from scipy.spatial.transform import Rotation as Rot

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "docs"))
from render_demo_video import ensure_assets, UMI_MESH_DIR, get_font

IMG_DIR = ROOT / "docs" / "img"
HOLES_YZ = [(-0.0004, -0.0150), (-0.0004, -0.0079), (-0.0004, -0.0221), (0.0067, -0.0150), (-0.0074, -0.0150)]
LEVER_HOLES_XZ = [(0.0071, 0.0), (-0.0071, 0.0), (0.0, 0.0071), (0.0, -0.0071)]


def render_hero():
    mesh_dir = str(UMI_MESH_DIR).replace("\\", "/")
    q_deg = 34.0
    q_rad = math.radians(q_deg)

    # Dynamic spring endpoints at q = 34 deg
    p_anc = np.array([-0.042, -0.0353, -0.026])
    c, s = math.cos(-q_rad), math.sin(-q_rad)
    p_tail = np.array([-0.016 * c + 0.024 * s, -0.0353, 0.016 * s + 0.024 * c])
    # Build coil spring rings along p_anc -> p_tail
    spring_geoms = []
    n_coils = 11
    for i in range(n_coils):
        t0 = i / n_coils
        t1 = (i + 1) / n_coils
        a = p_anc + (p_tail - p_anc) * t0
        b = p_anc + (p_tail - p_anc) * t1
        spring_geoms.append(
            f'<geom type="capsule" fromto="{a[0]:.5f} {a[1]:.5f} {a[2]:.5f} {b[0]:.5f} {b[1]:.5f} {b[2]:.5f}" size="0.0024" material="metal_mat"/>'
        )
    spring_xml = "\n        ".join(spring_geoms)

    # 4 M3 screws on lever hub (rotated with finger)
    lever_screws = "\n          ".join(
        f'<geom type="cylinder" pos="{dx:.4f} -0.0388 {dz:.4f}" euler="1.5708 0 0" size="0.0024 0.0014" material="metal_mat"/>'
        for dx, dz in LEVER_HOLES_XZ
    )

    xml = f"""
<mujoco model="sourccey_umi_hero">
  <compiler angle="radian" meshdir="{mesh_dir}"/>
  <visual>
    <global offwidth="1920" offheight="1080" fovy="32"/>
    <quality shadowsize="4096"/>
    <headlight ambient="0.32 0.33 0.35" diffuse="0.42 0.42 0.44" specular="0.22 0.22 0.22"/>
    <rgba haze="0.93 0.94 0.96 1"/>
    <map fogstart="0.8" fogend="2.2"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.95 0.96 0.98" rgb2="0.92 0.93 0.96" width="512" height="512"/>
    <material name="floor_mat" rgba="0.93 0.94 0.96 1" reflectance="0.06" roughness="0.60"/>
    <material name="base_mat" rgba="0.14 0.36 0.80 1" roughness="0.38" specular="0.38"/>
    <material name="finger_mat" rgba="0.96 0.46 0.12 1" roughness="0.38" specular="0.38"/>
    <material name="lever_mat" rgba="0.12 0.60 0.34 1" roughness="0.38" specular="0.38"/>
    <material name="grip_mat" rgba="0.22 0.24 0.28 1" roughness="0.52" specular="0.25"/>
    <material name="servo_mat" rgba="0.11 0.12 0.15 1" roughness="0.35" specular="0.45"/>
    <material name="metal_mat" rgba="0.80 0.82 0.86 1" roughness="0.20" specular="0.90" shininess="0.85"/>
    <material name="strap_mat" rgba="0.08 0.08 0.09 1" roughness="0.90" specular="0.05"/>
    <material name="cam_mat" rgba="0.10 0.11 0.13 1" roughness="0.32" specular="0.50"/>
    <material name="lens_mat" rgba="0.04 0.07 0.16 1" roughness="0.08" specular="0.95" shininess="0.95"/>
    <material name="block_mat" rgba="0.90 0.28 0.23 1" roughness="0.36" specular="0.25"/>

    <mesh name="umi_base" file="umi_base.stl"/>
    <mesh name="umi_finger" file="umi_finger.stl"/>
    <mesh name="umi_servo" file="umi_servo.stl"/>
    <mesh name="umi_grip" file="umi_grip.stl"/>
    <mesh name="umi_lever" file="umi_lever.stl"/>
  </asset>

  <worldbody>
    <light pos="0.45 -0.55 0.75" dir="-0.45 0.55 -0.8" diffuse="0.68 0.68 0.66" specular="0.35 0.35 0.35" castshadow="true"/>
    <light pos="-0.55 0.45 0.55" dir="0.55 -0.45 -0.6" diffuse="0.34 0.36 0.42" specular="0.18 0.18 0.18" castshadow="false"/>
    <light pos="0.10 0.60 0.35" dir="-0.1 -0.8 -0.4" diffuse="0.22 0.24 0.30" castshadow="false"/>

    <!-- Minimalist studio floor at z = -0.135 (exact bottom of pistol grip) -->
    <geom name="floor" type="plane" pos="0 0 -0.135" size="2 2 0.05" material="floor_mat"/>

    <!-- 38mm manipulation cube resting on workbench beside the jaws -->
    <geom type="box" pos="0.072 -0.015 -0.116" euler="0 0 0.38" size="0.019 0.019 0.019" material="block_mat"/>

    <!-- Sourccey UMI Assembly (exact pivot frame) -->
    <body name="umi" pos="0 0 0">
      <geom type="mesh" mesh="umi_base" material="base_mat"/>
      <geom type="mesh" mesh="umi_servo" material="servo_mat"/>
      <geom type="mesh" mesh="umi_grip" material="grip_mat"/>

      <!-- 20x20 mm Wrist Camera STRICTLY behind the Ø25 mm opening in the +y side wing (only camera needed) -->
      <body name="wrist_camera" pos="-0.014 0.052 -0.0175" euler="0 0 -0.38">
        <geom type="box" pos="-0.004 0 0" size="0.0035 0.010 0.010" material="cam_mat"/>
        <geom type="cylinder" pos="0.003 0 0" euler="0 1.5708 0" size="0.0068 0.0045" material="cam_mat"/>
        <geom type="cylinder" pos="0.0078 0 0" euler="0 1.5708 0" size="0.0048 0.0008" material="lens_mat"/>
      </body>

      <!-- Anchor pin + coil extension spring -->
      <geom type="cylinder" pos="-0.042 -0.020 -0.026" euler="1.5708 0 0" size="0.0017 0.036" material="metal_mat"/>
      {spring_xml}

      <!-- Hinged Finger + Trigger Lever at q = 34 deg -->
      <body name="hinge" pos="0 0 0">
        <joint name="q" type="hinge" axis="0 -1 0" range="-0.2 1.2"/>
        <geom type="mesh" mesh="umi_finger" material="finger_mat"/>
        <geom type="mesh" mesh="umi_lever" material="lever_mat"/>
        {lever_screws}
        <!-- Black velcro loop strap through the trigger pad side slots -->
        <geom type="capsule" fromto="-0.004 -0.042 -0.042  -0.018 -0.042 -0.042" size="0.0022" material="strap_mat"/>
        <geom type="capsule" fromto="-0.004 -0.030 -0.042  -0.018 -0.030 -0.042" size="0.0022" material="strap_mat"/>
        <geom type="capsule" fromto="-0.018 -0.042 -0.042  -0.024 -0.036 -0.042" size="0.0022" material="strap_mat"/>
        <geom type="capsule" fromto="-0.018 -0.030 -0.042  -0.024 -0.036 -0.042" size="0.0022" material="strap_mat"/>
      </body>
    </body>

    <!-- 3/4 front-side hero camera with clean studio headroom above pedestal and below cube -->
    <camera name="hero_cam" pos="0.22 -0.40 0.08" xyaxes="0.87 0.49 0 -0.14 0.24 0.96" fovy="34"/>
  </worldbody>
</mujoco>
"""
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    d.qpos[0] = q_rad
    mujoco.mj_forward(m, d)
    r = mujoco.Renderer(m, 1080, 1920)
    r.update_scene(d, camera="hero_cam")
    rgb = r.render()
    r.close()
    out = IMG_DIR / "00_hero.png"
    Image.fromarray(rgb).save(out)
    print("Saved CAD-exact hero render:", out)


def render_exploded():
    mesh_dir = str(UMI_MESH_DIR).replace("\\", "/")

    # Exploded offsets (meters) along true mechanical assembly axes:
    # Grip shifted left (-x = -0.105)
    # 5 brass inserts at -0.068, 5 M3x20 screws at -0.038 (EXACT 5-hole cross!)
    # Blue base at origin (0, 0, 0)
    # 20x20 wrist camera behind +y wing (+y = +0.055)
    # STS3215 servo lifted slightly above/back (+z = +0.052, +y = +0.010)
    # Orange finger lifted straight up (+z = +0.082)
    # Green trigger lever pulled forward in -y (-y = -0.062, +z = +0.010) + 4 M3x16 screws at -y = -0.095
    # M3x70 pin + spring aligned with grip beam
    inserts_xml = []
    screws5_xml = []
    lines_xml = []
    for y, z in HOLES_YZ:
        # Brass heat-set insert
        inserts_xml.append(
            f'<geom type="cylinder" pos="-0.118 {y:.4f} {z:.4f}" euler="0 1.5708 0" size="0.0024 0.0032" material="brass_mat"/>'
        )
        # M3x20 screw
        screws5_xml.append(
            f'<geom type="cylinder" pos="-0.068 {y:.4f} {z:.4f}" euler="0 1.5708 0" size="0.0015 0.0095" material="metal_mat"/>'
        )
        screws5_xml.append(
            f'<geom type="cylinder" pos="-0.057 {y:.4f} {z:.4f}" euler="0 1.5708 0" size="0.0028 0.0018" material="metal_mat"/>'
        )
        # Thin alignment guide line
        lines_xml.append(
            f'<geom type="capsule" fromto="-0.135 {y:.4f} {z:.4f} -0.030 {y:.4f} {z:.4f}" size="0.00045" material="guide_mat"/>'
        )

    screws4_xml = []
    for dx, dz in LEVER_HOLES_XZ:
        # 4x M3x16 screws aligned with lever 4-hole cross (lever at y_off=-0.062, z_off=0.0)
        screws4_xml.append(
            f'<geom type="cylinder" pos="{dx:.4f} -0.125 {dz:.4f}" euler="1.5708 0 0" size="0.0015 0.0075" material="metal_mat"/>'
        )
        screws4_xml.append(
            f'<geom type="cylinder" pos="{dx:.4f} -0.134 {dz:.4f}" euler="1.5708 0 0" size="0.0028 0.0018" material="metal_mat"/>'
        )
        lines_xml.append(
            f'<geom type="capsule" fromto="{dx:.4f} -0.132 {dz:.4f} {dx:.4f} -0.032 {dz:.4f}" size="0.00045" material="guide_mat"/>'
        )

    # Alignment guides for finger (z), servo (z), wrist cam (+y wing), and pin
    lines_xml.append('<geom type="capsule" fromto="0 0 0.015  0 0 0.080" size="0.0005" material="guide_mat"/>')
    lines_xml.append('<geom type="capsule" fromto="-0.014 0.052 -0.0175  -0.038 0.108 -0.0175" size="0.0005" material="guide_mat"/>')
    lines_xml.append('<geom type="capsule" fromto="-0.147 -0.095 -0.026  -0.147 -0.020 -0.026" size="0.0005" material="guide_mat"/>')

    xml = f"""
<mujoco model="sourccey_umi_exploded">
  <compiler angle="radian" meshdir="{mesh_dir}"/>
  <visual>
    <global offwidth="1920" offheight="1080" fovy="32"/>
    <quality shadowsize="4096"/>
    <headlight ambient="0.34 0.35 0.36" diffuse="0.45 0.45 0.45" specular="0.18 0.18 0.18"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="flat" rgb1="0.96 0.96 0.97" rgb2="0.96 0.96 0.97" width="64" height="64"/>
    <material name="base_mat" rgba="0.14 0.36 0.80 1" roughness="0.38" specular="0.35"/>
    <material name="finger_mat" rgba="0.96 0.46 0.12 1" roughness="0.38" specular="0.35"/>
    <material name="lever_mat" rgba="0.12 0.60 0.34 1" roughness="0.38" specular="0.35"/>
    <material name="grip_mat" rgba="0.24 0.26 0.30 1" roughness="0.50" specular="0.22"/>
    <material name="servo_mat" rgba="0.12 0.13 0.16 1" roughness="0.35" specular="0.45"/>
    <material name="metal_mat" rgba="0.72 0.75 0.80 1" roughness="0.22" specular="0.85" shininess="0.8"/>
    <material name="brass_mat" rgba="0.82 0.64 0.24 1" roughness="0.25" specular="0.80" shininess="0.7"/>
    <material name="cam_mat" rgba="0.10 0.11 0.13 1" roughness="0.32" specular="0.50"/>
    <material name="lens_mat" rgba="0.04 0.07 0.16 1" roughness="0.08" specular="0.95"/>
    <material name="guide_mat" rgba="0.45 0.50 0.58 0.7"/>

    <mesh name="umi_base" file="umi_base.stl"/>
    <mesh name="umi_finger" file="umi_finger.stl"/>
    <mesh name="umi_servo" file="umi_servo.stl"/>
    <mesh name="umi_grip" file="umi_grip.stl"/>
    <mesh name="umi_lever" file="umi_lever.stl"/>
  </asset>

  <worldbody>
    <light pos="0.4 -0.6 0.8" dir="-0.3 0.5 -0.8" diffuse="0.60 0.60 0.58" specular="0.30 0.30 0.30" castshadow="false"/>
    <light pos="-0.5 0.5 0.6" dir="0.5 -0.4 -0.6" diffuse="0.35 0.36 0.40" specular="0.15 0.15 0.15" castshadow="false"/>

    <!-- 1. Pistol Grip (exploded left by -0.105 m) -->
    <body name="grip_part" pos="-0.105 0 0">
      <geom type="mesh" mesh="umi_grip" material="grip_mat"/>
    </body>

    <!-- 2. 5x Brass M3 Heat-Set Inserts + 5x M3x20 Screws (exact 5-hole cross) -->
    {"".join(inserts_xml)}
    {"".join(screws5_xml)}

    <!-- 3. Real Sourccey Base (center) -->
    <body name="base_part" pos="0 0 0">
      <geom type="mesh" mesh="umi_base" material="base_mat"/>
    </body>

    <!-- 4. 20x20 mm Wrist Camera behind the +y Ø25 mm side wing -->
    <body name="cam_part" pos="-0.032 0.125 -0.0175" euler="0 0 -0.38">
      <geom type="box" pos="-0.004 0 0" size="0.0035 0.010 0.010" material="cam_mat"/>
      <geom type="cylinder" pos="0.003 0 0" euler="0 1.5708 0" size="0.0068 0.0045" material="cam_mat"/>
      <geom type="cylinder" pos="0.0078 0 0" euler="0 1.5708 0" size="0.0048 0.0008" material="lens_mat"/>
    </body>

    <!-- 5. Feetech STS3215 Servo (lifted slightly above cavity) -->
    <body name="servo_part" pos="0.0 0.0 0.056">
      <geom type="mesh" mesh="umi_servo" material="servo_mat"/>
    </body>

    <!-- 6. Real Sourccey Hinged Finger (lifted up) -->
    <body name="finger_part" pos="0.0 0.0 0.088">
      <geom type="mesh" mesh="umi_finger" material="finger_mat"/>
    </body>

    <!-- 7. Emerald-Green Trigger Lever (exploded forward in -y) + 4x M3x16 screws -->
    <body name="lever_part" pos="0.0 -0.062 0.0">
      <geom type="mesh" mesh="umi_lever" material="lever_mat"/>
    </body>
    {"".join(screws4_xml)}

    <!-- 8. M3x70 Pin + Extension Spring (exploded along pin axis) -->
    <geom type="cylinder" pos="-0.147 -0.092 -0.026" euler="1.5708 0 0" size="0.0017 0.035" material="metal_mat"/>
    <geom type="capsule" fromto="-0.078 -0.095 -0.010  -0.045 -0.095 0.018" size="0.0030" material="metal_mat"/>

    <!-- Subtle alignment axes -->
    {"".join(lines_xml)}

    <camera name="exp_cam" pos="0.16 -0.50 0.16" xyaxes="0.93 0.36 0 -0.11 0.28 0.95" fovy="35"/>
  </worldbody>
</mujoco>
"""
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    r = mujoco.Renderer(m, 1080, 1920)
    r.update_scene(d, camera="exp_cam")
    rgb = r.render()
    r.close()

    # Add crisp, minimal architectural legend strip at bottom so every part is unmistakable
    img = Image.fromarray(rgb)
    draw = ImageDraw.Draw(img)
    W, H = img.size
    draw.rectangle([0, H - 72, W, H], fill=(244, 245, 248))
    draw.line([(0, H - 72), (W, H - 72)], fill=(215, 220, 228), width=2)

    items = [
        ((60, 66, 76), "Pistol Grip + 1/4-20 Mount"),
        ((209, 163, 61), "5x M3 Inserts & M3x20"),
        ((36, 92, 204), "Sourccey Base (+y Cam Wing)"),
        ((26, 28, 34), "20x20 Wrist Cam & STS3215"),
        ((245, 117, 31), "Sourccey Finger"),
        ((31, 153, 87), "Trigger Lever + 4x M3x16 + Spring"),
    ]
    font = get_font(20, bold=True)
    x = 48
    for col, label in items:
        draw.rounded_rectangle([x, H - 46, x + 20, H - 26], radius=4, fill=col)
        draw.text((x + 28, H - 48), label, fill=(30, 41, 59), font=font)
        x += int(draw.textlength(label, font=font)) + 58

    out = IMG_DIR / "00_exploded_overview.png"
    img.save(out)
    print("Saved CAD-exact exploded view:", out)


if __name__ == "__main__":
    ensure_assets()
    render_hero()
    render_exploded()
