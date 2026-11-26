"""Render the Sourccey UMI -> Sourccey Robot pick-and-place demonstration video.

Generates:
  - docs/media/sourccey_umi_demo.mp4  (1280x720, 30 fps)
  - docs/media/sourccey_umi_demo.gif  (640x360, 12 fps preview for GitHub README)

Usage:
  python docs/render_demo_video.py --preview   # fast 2x2 keyframe sheet for inspection
  python docs/render_demo_video.py             # full video + GIF render
"""
import argparse
import math
import pathlib
import sys
import urllib.request

import cv2
import mujoco
import numpy as np
import trimesh
from PIL import Image, ImageDraw, ImageFont
from scipy.spatial.transform import Rotation as Rot

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "software"))

import common as C
from retarget_ik import SourcceyArm, T_GB_TOOL

CACHE_DIR = ROOT / ".cache" / "sourccey"
ARM_DIR = CACHE_DIR / "ArmLeft"
UMI_MESH_DIR = CACHE_DIR / "umi_meshes"
MEDIA_DIR = ROOT / "docs" / "media"

# ---------------------------------------------------------------------------
# 1. Asset preparation (real Sourccey arm + aligned UMI meshes in meters)
# ---------------------------------------------------------------------------
def ensure_assets():
    UMI_MESH_DIR.mkdir(parents=True, exist_ok=True)
    MEDIA_DIR.mkdir(parents=True, exist_ok=True)

    # Real Sourccey gripper parts in pivot frame (mm -> m)
    real = C.load_pivot_frame("L", ("base", "finger", "servo"))
    for k, mesh in real.items():
        m = mesh.copy()
        m.apply_scale(0.001)
        m.export(UMI_MESH_DIR / f"umi_{k}.stl")

    # Recover exact pivot-frame pose of pistol_grip_LEFT and trigger_lever_LEFT
    specs = [
        (
            "cad/stl/pistol_grip_LEFT.stl",
            "umi_grip.stl",
            1, 1,
            np.array([-66.0, -52.0, -135.0]),
        ),
        (
            "cad/stl/trigger_lever_LEFT.stl",
            "umi_lever.stl",
            2, 1,
            np.array([-23.0, -46.0, -53.0]),
        ),
    ]
    for src_rel, dst_name, axis, sign, bmin in specs:
        mesh = trimesh.load(ROOT / src_rel)
        v = np.zeros(3)
        v[axis] = -sign
        T = trimesh.geometry.align_vectors(v, [0, 0, -1])
        mesh.apply_transform(np.linalg.inv(T))
        mesh.apply_translation(bmin - mesh.bounds[0])
        mesh.apply_scale(0.001)
        mesh.export(UMI_MESH_DIR / dst_name)

    # Ensure ArmLeft URDF + all 20 meshes exist
    urdf_src = pathlib.Path(C.default_urdf())
    (ARM_DIR / "meshes").mkdir(parents=True, exist_ok=True)
    urdf_txt = urdf_src.read_text()
    (ARM_DIR / "ArmLeft.urdf").write_text(urdf_txt)
    import re
    meshes = sorted(set(re.findall(r'filename="([^"]+)"', urdf_txt)))
    base_url = "https://raw.githubusercontent.com/vulcan-forge/sourccey-hardware/main/URDF/Arms/ArmLeft/"
    for rel in meshes:
        dst = ARM_DIR / rel
        if not dst.exists():
            with urllib.request.urlopen(base_url + rel, timeout=60) as r:
                dst.write_bytes(r.read())


# ---------------------------------------------------------------------------
# 2. Trajectory helper (smoothstep quintic interpolation)
# ---------------------------------------------------------------------------
def smooth(u):
    u = np.clip(u, 0.0, 1.0)
    return u * u * u * (u * (6.0 * u - 15.0) + 10.0)


def lerp(a, b, u):
    return a + (b - a) * smooth(u)


# Block & table geometry (world frame in meters)
# Raised sleek task platform at z = 0.095 so the handheld pistol grip clears the table naturally
PLATFORM_H = 0.095
BLOCK_HALF = 0.019
PICK_XY = np.array([0.04, -0.085])
PLACE_XY = np.array([0.04, 0.085])
# In pivot frame, lower jaw interior rests at z = -0.026 m, jaw tips at x = +0.070 m.
GRASP_OFFSET = np.array([-0.062, 0.0, 0.020])
Q_OPEN_DEG = 56.0
Q_CLOSED_DEG = 21.0  # snugly cradles 38mm cube without clipping


def sample_task_trajectory(n_frames):
    """Return per-frame (tool_pos[3], q_deg, block_pos[3], phase_label, carrying)."""
    bz = PLATFORM_H + BLOCK_HALF
    p_pick_hi = np.array([PICK_XY[0], PICK_XY[1], bz]) + GRASP_OFFSET + np.array([0, 0, 0.085])
    p_pick = np.array([PICK_XY[0], PICK_XY[1], bz]) + GRASP_OFFSET
    p_lift = p_pick + np.array([0, 0, 0.085])
    p_place_hi = np.array([PLACE_XY[0], PLACE_XY[1], bz]) + GRASP_OFFSET + np.array([0, 0, 0.085])
    p_place = np.array([PLACE_XY[0], PLACE_XY[1], bz]) + GRASP_OFFSET
    p_home = np.array([0.00, -0.04, PLATFORM_H + 0.14])

    block_pick = np.array([PICK_XY[0], PICK_XY[1], bz])
    block_place = np.array([PLACE_XY[0], PLACE_XY[1], bz])

    traj = []
    for i in range(n_frames):
        t = i / max(1, n_frames - 1)
        if t < 0.14:
            u = t / 0.14
            pos = lerp(p_home, p_pick_hi, u)
            q = Q_OPEN_DEG
            bpos = block_pick.copy()
            label = "APPROACH"
            carry = False
        elif t < 0.28:
            u = (t - 0.14) / 0.14
            pos = lerp(p_pick_hi, p_pick, u)
            q = Q_OPEN_DEG
            bpos = block_pick.copy()
            label = "DESCEND"
            carry = False
        elif t < 0.40:
            u = (t - 0.28) / 0.12
            pos = p_pick.copy()
            q = lerp(Q_OPEN_DEG, Q_CLOSED_DEG, u)
            bpos = block_pick.copy()
            label = "GRASP"
            carry = u > 0.85
        elif t < 0.54:
            u = (t - 0.40) / 0.14
            pos = lerp(p_pick, p_lift, u)
            q = Q_CLOSED_DEG
            bpos = pos - GRASP_OFFSET
            label = "LIFT"
            carry = True
        elif t < 0.72:
            u = (t - 0.54) / 0.18
            pos = lerp(p_lift, p_place_hi, u)
            # slight graceful arc
            pos[2] += 0.025 * math.sin(math.pi * smooth(u))
            q = Q_CLOSED_DEG
            bpos = pos - GRASP_OFFSET
            label = "TRANSPORT"
            carry = True
        elif t < 0.84:
            u = (t - 0.72) / 0.12
            pos = lerp(p_place_hi, p_place, u)
            q = Q_CLOSED_DEG
            bpos = pos - GRASP_OFFSET
            label = "PLACE"
            carry = True
        elif t < 0.92:
            u = (t - 0.84) / 0.08
            pos = p_place.copy()
            q = lerp(Q_CLOSED_DEG, Q_OPEN_DEG, u)
            bpos = block_place.copy()
            label = "RELEASE"
            carry = False
        else:
            u = (t - 0.92) / 0.08
            pos = lerp(p_place, p_place_hi, u)
            q = Q_OPEN_DEG
            bpos = block_place.copy()
            label = "RETRACT"
            carry = False
        traj.append((pos, q, bpos, label, carry))
    return traj


# ---------------------------------------------------------------------------
# 3. Scene 1: Human Hand + Sourccey UMI (MuJoCo MJCF)
# ---------------------------------------------------------------------------
def build_umi_scene_xml():
    mesh_dir = str(UMI_MESH_DIR).replace("\\", "/")
    return f"""
<mujoco model="sourccey_umi_human">
  <compiler angle="radian" meshdir="{mesh_dir}"/>
  <visual>
    <global offwidth="1280" offheight="720" fovy="42"/>
    <quality shadowsize="4096"/>
    <headlight ambient="0.22 0.23 0.25" diffuse="0.32 0.32 0.34" specular="0.18 0.18 0.18"/>
    <rgba haze="0.88 0.90 0.94 1"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.86 0.89 0.93" rgb2="0.68 0.73 0.80" width="512" height="512"/>
    <texture name="grid_tex" type="2d" builtin="checker" rgb1="0.74 0.76 0.79" rgb2="0.69 0.71 0.75" width="512" height="512" mark="edge" markrgb="0.62 0.65 0.70"/>
    <material name="table_mat" texture="grid_tex" texrepeat="12 12" reflectance="0.06" roughness="0.62"/>
    <material name="base_mat" rgba="0.18 0.36 0.78 1" roughness="0.42" specular="0.35"/>
    <material name="finger_mat" rgba="0.95 0.47 0.14 1" roughness="0.42" specular="0.35"/>
    <material name="lever_mat" rgba="0.14 0.62 0.34 1" roughness="0.42" specular="0.35"/>
    <material name="grip_mat" rgba="0.22 0.24 0.28 1" roughness="0.55" specular="0.25"/>
    <material name="servo_mat" rgba="0.12 0.13 0.16 1" roughness="0.35" specular="0.45"/>
    <material name="metal_mat" rgba="0.78 0.80 0.84 1" roughness="0.22" specular="0.85" shininess="0.8"/>
    <material name="skin_mat" rgba="0.86 0.67 0.55 1" roughness="0.65" specular="0.12"/>
    <material name="sleeve_mat" rgba="0.18 0.22 0.29 1" roughness="0.8" specular="0.05"/>
    <material name="block_mat" rgba="0.90 0.26 0.22 1" roughness="0.35" specular="0.3"/>
    <material name="pad_pick_mat" rgba="0.90 0.26 0.22 0.22" roughness="0.8"/>
    <material name="pad_place_mat" rgba="0.16 0.68 0.42 0.28" roughness="0.8"/>
    <material name="cam_mat" rgba="0.10 0.11 0.13 1" roughness="0.35" specular="0.5"/>
    <material name="lens_mat" rgba="0.05 0.08 0.18 1" roughness="0.1" specular="0.95"/>

    <mesh name="umi_base" file="umi_base.stl"/>
    <mesh name="umi_finger" file="umi_finger.stl"/>
    <mesh name="umi_servo" file="umi_servo.stl"/>
    <mesh name="umi_grip" file="umi_grip.stl"/>
    <mesh name="umi_lever" file="umi_lever.stl"/>
  </asset>

  <worldbody>
    <light pos="0.5 -0.6 1.2" dir="-0.4 0.4 -1" diffuse="0.62 0.62 0.60" specular="0.30 0.30 0.30" castshadow="true"/>
    <light pos="-0.6 0.5 0.9" dir="0.5 -0.4 -0.8" diffuse="0.32 0.34 0.40" specular="0.15 0.15 0.15" castshadow="false"/>

    <!-- Workbench + Raised Task Platform -->
    <geom name="table" type="plane" pos="0 0 0" size="1.5 1.5 0.05" material="table_mat"/>
    <geom name="platform" type="box" pos="0.06 0.0 {PLATFORM_H/2}" size="0.095 0.155 {PLATFORM_H/2}" rgba="0.86 0.87 0.90 1"/>
    <!-- Pick & Place zone pads -->
    <geom name="pick_pad" type="box" pos="{PICK_XY[0]} {PICK_XY[1]} {PLATFORM_H + 0.0005}" size="0.034 0.034 0.0005" material="pad_pick_mat"/>
    <geom name="place_pad" type="box" pos="{PLACE_XY[0]} {PLACE_XY[1]} {PLATFORM_H + 0.0005}" size="0.034 0.034 0.0005" material="pad_place_mat"/>

    <!-- Manipulation Block (mocap for crisp synced animation) -->
    <body name="block" mocap="true" pos="{PICK_XY[0]} {PICK_XY[1]} {PLATFORM_H + BLOCK_HALF}">
      <geom type="box" size="{BLOCK_HALF} {BLOCK_HALF} {BLOCK_HALF}" material="block_mat"/>
    </body>

    <!-- Handheld Sourccey UMI + Articulated Hand (mocap root at finger pivot) -->
    <body name="umi_root" mocap="true" pos="0 -0.09 0.14">
      <geom type="mesh" mesh="umi_base" material="base_mat"/>
      <geom type="mesh" mesh="umi_servo" material="servo_mat"/>
      <geom type="mesh" mesh="umi_grip" material="grip_mat"/>

      <!-- 20x20 mm Wrist Camera in the base side wing -->
      <geom type="box" pos="-0.014 0.052 -0.0175" size="0.006 0.010 0.010" material="cam_mat"/>
      <geom type="cylinder" pos="-0.006 0.049 -0.0175" euler="0 1.57 0" size="0.0065 0.004" material="lens_mat"/>
      <camera name="wrist_cam" pos="-0.006 0.049 -0.0175" xyaxes="-0.42 -0.91 0 -0.10 0.05 0.99" fovy="64"/>

      <!-- GoPro-style Pose Tracker on the top 1/4-20 pedestal -->
      <geom type="cylinder" pos="-0.051 -0.007 0.038" size="0.005 0.006" material="metal_mat"/>
      <geom type="box" pos="-0.051 -0.007 0.058" size="0.015 0.031 0.022" material="cam_mat"/>
      <geom type="cylinder" pos="-0.035 0.006 0.060" euler="0 1.57 0" size="0.011 0.004" material="lens_mat"/>

      <!-- Anchor pin for extension spring -->
      <geom type="cylinder" pos="-0.042 -0.020 -0.026" euler="1.5708 0 0" size="0.0018 0.036" material="metal_mat"/>

      <!-- Dynamic spring placeholder (updated via mocap body below) -->

      <!-- Rotating Finger + Trigger Lever -->
      <body name="umi_hinge" pos="0 0 0">
        <joint name="umi_q" type="hinge" axis="0 -1 0" range="-0.2 1.2"/>
        <geom type="mesh" mesh="umi_finger" material="finger_mat"/>
        <geom type="mesh" mesh="umi_lever" material="lever_mat"/>
        <!-- M3 cross screws on lever hub -->
        <geom type="cylinder" pos="0.0071 -0.0385 0" euler="1.5708 0 0" size="0.0022 0.0012" material="metal_mat"/>
        <geom type="cylinder" pos="-0.0071 -0.0385 0" euler="1.5708 0 0" size="0.0022 0.0012" material="metal_mat"/>
        <geom type="cylinder" pos="0 -0.0385 0.0071" euler="1.5708 0 0" size="0.0022 0.0012" material="metal_mat"/>
        <geom type="cylinder" pos="0 -0.0385 -0.0071" euler="1.5708 0 0" size="0.0022 0.0012" material="metal_mat"/>
      </body>

      <!-- Articulated Human Hand holding the pistol grip -->
      <body name="human_hand" pos="0 0 0">
        <!-- Palm wrapping the rear & sides of the grip (x in [-0.095..-0.060], y around -0.037, z in [-0.115..-0.042]) -->
        <geom type="capsule" fromto="-0.080 -0.056 -0.056  -0.082 -0.056 -0.106" size="0.017" material="skin_mat"/>
        <geom type="capsule" fromto="-0.076 -0.022 -0.058  -0.078 -0.022 -0.106" size="0.015" material="skin_mat"/>
        <geom type="capsule" fromto="-0.084 -0.039 -0.056  -0.086 -0.039 -0.108" size="0.019" material="skin_mat"/>
        <!-- Wrist & Forearm extending smoothly back-down-left -->
        <geom type="capsule" fromto="-0.095 -0.042 -0.085  -0.165 -0.055 -0.105" size="0.025" material="skin_mat"/>
        <geom type="capsule" fromto="-0.165 -0.055 -0.105  -0.320 -0.080 -0.145" size="0.032" material="sleeve_mat"/>

        <!-- Thumb curving around the +y side of the upper grip -->
        <geom type="capsule" fromto="-0.074 -0.022 -0.058  -0.052 -0.014 -0.048" size="0.0105" material="skin_mat"/>
        <geom type="capsule" fromto="-0.052 -0.014 -0.048  -0.032 -0.018 -0.046" size="0.0090" material="skin_mat"/>

        <!-- Middle finger (groove z = -0.070) -->
        <geom type="capsule" fromto="-0.072 -0.056 -0.068  -0.044 -0.058 -0.070" size="0.0092" material="skin_mat"/>
        <geom type="capsule" fromto="-0.044 -0.058 -0.070  -0.026 -0.046 -0.070" size="0.0082" material="skin_mat"/>
        <geom type="capsule" fromto="-0.026 -0.046 -0.070  -0.028 -0.026 -0.070" size="0.0074" material="skin_mat"/>

        <!-- Ring finger (groove z = -0.090) -->
        <geom type="capsule" fromto="-0.074 -0.056 -0.088  -0.046 -0.057 -0.090" size="0.0086" material="skin_mat"/>
        <geom type="capsule" fromto="-0.046 -0.057 -0.090  -0.027 -0.045 -0.090" size="0.0078" material="skin_mat"/>
        <geom type="capsule" fromto="-0.027 -0.045 -0.090  -0.029 -0.027 -0.090" size="0.0070" material="skin_mat"/>

        <!-- Pinky finger (groove z = -0.110) -->
        <geom type="capsule" fromto="-0.076 -0.054 -0.108  -0.048 -0.054 -0.110" size="0.0078" material="skin_mat"/>
        <geom type="capsule" fromto="-0.048 -0.054 -0.110  -0.030 -0.044 -0.110" size="0.0070" material="skin_mat"/>
        <geom type="capsule" fromto="-0.030 -0.044 -0.110  -0.031 -0.029 -0.110" size="0.0064" material="skin_mat"/>

        <!-- Index finger (knuckle at -0.068, -0.055, -0.048) articulated to pull the trigger! -->
        <body name="index_prox" pos="-0.068 -0.055 -0.048">
          <joint name="idx_j1" type="hinge" axis="0 0 1" range="-1.0 1.0"/>
          <geom type="capsule" fromto="0 0 0  0.038 -0.003 0.003" size="0.0090" material="skin_mat"/>
          <body name="index_mid" pos="0.038 -0.003 0.003">
            <joint name="idx_j2" type="hinge" axis="0 0 1" range="-1.5 1.5"/>
            <geom type="capsule" fromto="0 0 0  0.028 0.012 0.002" size="0.0080" material="skin_mat"/>
            <body name="index_tip" pos="0.028 0.012 0.002">
              <joint name="idx_j3" type="hinge" axis="0 0 1" range="-1.5 1.5"/>
              <geom type="capsule" fromto="0 0 0  0.012 0.014 0.000" size="0.0072" material="skin_mat"/>
            </body>
          </body>
        </body>
      </body>
    </body>

    <!-- Dynamic metallic spring (mocap capsule positioned every frame) -->
    <body name="spring_body" mocap="true" pos="0 0 0.5">
      <geom name="spring_geom" type="capsule" size="0.0028 0.020" material="metal_mat"/>
    </body>

    <camera name="main_cam" pos="0.42 -0.42 0.34" xyaxes="0.72 0.70 0 -0.32 0.33 0.89" fovy="44"/>
  </worldbody>
</mujoco>
"""


# ---------------------------------------------------------------------------
# 4. Scene 2: Real Sourccey Left Arm (MuJoCo MJCF)
# ---------------------------------------------------------------------------
def build_robot_scene_xml():
    arm_dir = str(ARM_DIR).replace("\\", "/")
    # Mount ArmLeft base_link on a sleek pedestal behind the table (at x=-0.30, y=0.0, z=0.0)
    # Note: In ArmLeft.urdf, base_link sits around (0.30, 0.27, -0.02) and arm reaches along -y!
    # We rotate the whole robot base by -90 deg around Z so the arm reaches along +x right onto our table!
    return f"""
<mujoco model="sourccey_arm_replay">
  <compiler angle="radian" meshdir="{arm_dir}"/>
  <visual>
    <global offwidth="1280" offheight="720" fovy="42"/>
    <quality shadowsize="4096"/>
    <headlight ambient="0.22 0.23 0.25" diffuse="0.32 0.32 0.34" specular="0.18 0.18 0.18"/>
    <rgba haze="0.88 0.90 0.94 1"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.86 0.89 0.93" rgb2="0.68 0.73 0.80" width="512" height="512"/>
    <texture name="grid_tex" type="2d" builtin="checker" rgb1="0.74 0.76 0.79" rgb2="0.69 0.71 0.75" width="512" height="512" mark="edge" markrgb="0.62 0.65 0.70"/>
    <material name="table_mat" texture="grid_tex" texrepeat="12 12" reflectance="0.06" roughness="0.62"/>
    <material name="arm_mat" rgba="0.26 0.28 0.33 1" roughness="0.45" specular="0.35"/>
    <material name="arm_accent" rgba="0.88 0.89 0.92 1" roughness="0.38" specular="0.4"/>
    <material name="servo_mat" rgba="0.12 0.13 0.16 1" roughness="0.35" specular="0.45"/>
    <material name="base_mat" rgba="0.18 0.36 0.78 1" roughness="0.42" specular="0.35"/>
    <material name="finger_mat" rgba="0.95 0.47 0.14 1" roughness="0.42" specular="0.35"/>
    <material name="block_mat" rgba="0.90 0.26 0.22 1" roughness="0.35" specular="0.3"/>
    <material name="pad_pick_mat" rgba="0.90 0.26 0.22 0.22" roughness="0.8"/>
    <material name="pad_place_mat" rgba="0.16 0.68 0.42 0.28" roughness="0.8"/>
    <material name="cam_mat" rgba="0.10 0.11 0.13 1" roughness="0.35" specular="0.5"/>
    <material name="lens_mat" rgba="0.05 0.08 0.18 1" roughness="0.1" specular="0.95"/>

    <mesh name="base_link" file="meshes/base_link.stl" scale="0.001 0.001 0.001"/>
    <mesh name="Feetech_Servo_Motor_v1_1" file="meshes/Feetech_Servo_Motor_v1_1.stl" scale="0.001 0.001 0.001"/>
    <mesh name="Motor_1_Support_v1_1" file="meshes/Motor_1_Support_v1_1.stl" scale="0.001 0.001 0.001"/>
    <mesh name="Shoulder_v1_1" file="meshes/Shoulder_v1_1.stl" scale="0.001 0.001 0.001"/>
    <mesh name="Feetech_Servo_Motor_v1_2" file="meshes/Feetech_Servo_Motor_v1_2.stl" scale="0.001 0.001 0.001"/>
    <mesh name="Motor_2_Support_v1_1" file="meshes/Motor_2_Support_v1_1.stl" scale="0.001 0.001 0.001"/>
    <mesh name="Bicep_Left_v1_1" file="meshes/Bicep_Left_v1_1.stl" scale="0.001 0.001 0.001"/>
    <mesh name="Bicep_Right_v1_1" file="meshes/Bicep_Right_v1_1.stl" scale="0.001 0.001 0.001"/>
    <mesh name="Feetech_Servo_Motor_v1_3" file="meshes/Feetech_Servo_Motor_v1_3.stl" scale="0.001 0.001 0.001"/>
    <mesh name="Motor_3_support_v1_1" file="meshes/Motor_3_support_v1_1.stl" scale="0.001 0.001 0.001"/>
    <mesh name="Forearm_v1_1" file="meshes/Forearm_v1_1.stl" scale="0.001 0.001 0.001"/>
    <mesh name="Feetech_Servo_Motor_v1_4" file="meshes/Feetech_Servo_Motor_v1_4.stl" scale="0.001 0.001 0.001"/>
    <mesh name="Motor_4_support_v1_1" file="meshes/Motor_4_support_v1_1.stl" scale="0.001 0.001 0.001"/>
    <mesh name="Wrist_v1_1" file="meshes/Wrist_v1_1.stl" scale="0.001 0.001 0.001"/>
    <mesh name="Motor_5_support_v1_1" file="meshes/Motor_5_support_v1_1.stl" scale="0.001 0.001 0.001"/>
    <mesh name="Feetech_Servo_Motor_v1_5" file="meshes/Feetech_Servo_Motor_v1_5.stl" scale="0.001 0.001 0.001"/>
    <mesh name="Gripper_Base_v1_1" file="meshes/Gripper_Base_v1_1.stl" scale="0.001 0.001 0.001"/>
    <mesh name="Feetech_Servo_Motor_v1_6" file="meshes/Feetech_Servo_Motor_v1_6.stl" scale="0.001 0.001 0.001"/>
    <mesh name="Motor_6_cap_v2_1" file="meshes/Motor_6_cap_v2_1.stl" scale="0.001 0.001 0.001"/>
    <mesh name="Gripper_Finger_v1_1" file="meshes/Gripper_Finger_v1_1.stl" scale="0.001 0.001 0.001"/>
  </asset>

  <worldbody>
    <light pos="0.5 -0.6 1.2" dir="-0.4 0.4 -1" diffuse="0.62 0.62 0.60" specular="0.30 0.30 0.30" castshadow="true"/>
    <light pos="-0.6 0.5 0.9" dir="0.5 -0.4 -0.8" diffuse="0.32 0.34 0.40" specular="0.15 0.15 0.15" castshadow="false"/>

    <!-- Workbench + Raised Task Platform -->
    <geom name="table" type="plane" pos="0 0 0" size="1.5 1.5 0.05" material="table_mat"/>
    <geom name="platform" type="box" pos="0.06 0.0 {PLATFORM_H/2}" size="0.095 0.155 {PLATFORM_H/2}" rgba="0.86 0.87 0.90 1"/>
    <geom name="pick_pad" type="box" pos="{PICK_XY[0]} {PICK_XY[1]} {PLATFORM_H + 0.0005}" size="0.034 0.034 0.0005" material="pad_pick_mat"/>
    <geom name="place_pad" type="box" pos="{PLACE_XY[0]} {PLACE_XY[1]} {PLATFORM_H + 0.0005}" size="0.034 0.034 0.0005" material="pad_place_mat"/>

    <!-- Sleek Sourccey Torso/Shoulder Mount Pedestal -->
    <geom type="box" pos="-0.39 0.0 0.17" size="0.055 0.075 0.17" material="arm_mat"/>

    <!-- Manipulation Block -->
    <body name="block" mocap="true" pos="{PICK_XY[0]} {PICK_XY[1]} {PLATFORM_H + BLOCK_HALF}">
      <geom type="box" size="{BLOCK_HALF} {BLOCK_HALF} {BLOCK_HALF}" material="block_mat"/>
    </body>

    <!-- Real Sourccey Left Arm mounted at world pose T_world_base -->
    <!-- Rotated +90 deg about Z (euler 0 0 1.5708): arm base_link (-y forward) now points along +x! -->
    <body name="robot_mount" pos="-0.10 -0.299 0.34" euler="0 0 1.57079632679">
      <geom pos="0 0 0" type="mesh" mesh="base_link" material="arm_mat"/>
      <geom pos="0 0 0" type="mesh" mesh="Feetech_Servo_Motor_v1_1" material="servo_mat"/>
      <geom pos="0 0 0" type="mesh" mesh="Motor_1_Support_v1_1" material="arm_accent"/>
      <body name="Arm-Base-Shoulder" pos="0.299356 0.244233 0.224234">
        <joint name="shoulder_pan" range="-1.5708 1.5708"/>
        <geom pos="-0.299356 -0.244233 -0.224234" type="mesh" mesh="Shoulder_v1_1" material="arm_accent"/>
        <geom pos="-0.299356 -0.244233 -0.224234" type="mesh" mesh="Feetech_Servo_Motor_v1_2" material="servo_mat"/>
        <geom pos="-0.299356 -0.244233 -0.224234" type="mesh" mesh="Motor_2_Support_v1_1" material="arm_mat"/>
        <body name="Arm-Bicep" pos="0.192873 -0.07 -0.244232">
          <joint name="shoulder_lift" range="-1.5708 1.5708" axis="-1 0 0"/>
          <geom pos="-0.492229 -0.174233 0.019998" type="mesh" mesh="Bicep_Left_v1_1" material="arm_accent"/>
          <geom pos="-0.492229 -0.174233 0.019998" type="mesh" mesh="Bicep_Right_v1_1" material="arm_accent"/>
          <geom pos="-0.492229 -0.174233 0.019998" type="mesh" mesh="Feetech_Servo_Motor_v1_3" material="servo_mat"/>
          <geom pos="-0.492229 -0.174233 0.019998" type="mesh" mesh="Motor_3_support_v1_1" material="arm_mat"/>
          <body name="Arm-Forearm" pos="-0.193023 -0.15711 0">
            <joint name="elbow_flex" range="-1.5708 1.5708" axis="-1 0 0"/>
            <geom pos="-0.299206 -0.017123 0.019998" type="mesh" mesh="Forearm_v1_1" material="arm_accent"/>
            <geom pos="-0.299206 -0.017123 0.019998" type="mesh" mesh="Feetech_Servo_Motor_v1_4" material="servo_mat"/>
            <geom pos="-0.299206 -0.017123 0.019998" type="mesh" mesh="Motor_4_support_v1_1" material="arm_mat"/>
            <body name="Arm-Wrist" pos="0.000375 -0.161475 0">
              <joint name="wrist_flex" range="-1.5708 1.5708" axis="1 0 0"/>
              <geom pos="-0.299581 0.144352 0.019998" type="mesh" mesh="Wrist_v1_1" material="arm_accent"/>
              <geom pos="-0.299581 0.144352 0.019998" type="mesh" mesh="Motor_5_support_v1_1" material="arm_mat"/>
              <geom pos="-0.299581 0.144352 0.019998" type="mesh" mesh="Feetech_Servo_Motor_v1_5" material="servo_mat"/>
              <body name="Gripper-Base" pos="0.0013 -0.07025 0">
                <joint name="wrist_roll" range="-3.14159 3.14159" axis="0 -1 0"/>
                <geom pos="-0.300881 0.214602 0.019998" type="mesh" mesh="Gripper_Base_v1_1" material="base_mat"/>
                <geom pos="-0.300881 0.214602 0.019998" type="mesh" mesh="Feetech_Servo_Motor_v1_6" material="servo_mat"/>
                <geom pos="-0.300881 0.214602 0.019998" type="mesh" mesh="Motor_6_cap_v2_1" material="arm_mat"/>
                <!-- Identical Wrist Camera on Sourccey Gripper-Base (transformed via T_GB_TOOL) -->
                <body name="tool_frame" pos="0.000375 -0.030375 0.015" euler="0 0 -1.57079632679">
                  <geom type="box" pos="-0.014 0.052 -0.0175" size="0.006 0.010 0.010" material="cam_mat"/>
                  <geom type="cylinder" pos="-0.006 0.049 -0.0175" euler="0 1.57 0" size="0.0065 0.004" material="lens_mat"/>
                  <camera name="wrist_cam" pos="-0.006 0.049 -0.0175" xyaxes="-0.42 -0.91 0 -0.10 0.05 0.99" fovy="64"/>
                </body>
                <body name="Gripper-Finger" pos="0.000375 -0.030375 0.015">
                  <joint name="gripper" range="-1.5708 1.5708" axis="-1 0 0"/>
                  <geom pos="-0.301256 0.244977 0.004998" type="mesh" mesh="Gripper_Finger_v1_1" material="finger_mat"/>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>
    </body>

    <camera name="main_cam" pos="0.46 -0.46 0.40" xyaxes="0.72 0.69 0 -0.34 0.36 0.87" fovy="46"/>
  </worldbody>
</mujoco>
"""


# ---------------------------------------------------------------------------
# 5. HUD & Card Compositing (PIL)
# ---------------------------------------------------------------------------
def get_font(size, bold=False):
    candidates = [
        "C:/Windows/Fonts/segoeuib.ttf" if bold else "C:/Windows/Fonts/segoeui.ttf",
        "C:/Windows/Fonts/arialbd.ttf" if bold else "C:/Windows/Fonts/arial.ttf",
    ]
    for p in candidates:
        if pathlib.Path(p).exists():
            return ImageFont.truetype(p, size)
    return ImageFont.load_default()


def make_card(w, h, title, subtitle, tag="", accent=(56, 130, 246)):
    img = Image.new("RGB", (w, h), (15, 20, 30))
    draw = ImageDraw.Draw(img)
    # Subtle accent bar
    draw.rectangle([w // 2 - 120, h // 2 - 96, w // 2 + 120, h // 2 - 91], fill=accent)
    if tag:
        ftag = get_font(20, bold=True)
        draw.text((w // 2, h // 2 - 125), tag, fill=(148, 163, 184), font=ftag, anchor="mm")
    ftitle = get_font(56, bold=True)
    fsub = get_font(26, bold=False)
    draw.text((w // 2, h // 2 - 22), title, fill=(248, 250, 252), font=ftitle, anchor="mm")
    draw.text((w // 2, h // 2 + 46), subtitle, fill=(186, 198, 214), font=fsub, anchor="mm")
    return np.array(img)


def composite_hud(main_rgb, pip_rgb, phase_badge, accent, step_label, q_deg, extra_lines=()):
    img = Image.fromarray(main_rgb).convert("RGBA")
    overlay = Image.new("RGBA", img.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(overlay)
    W, H = img.size

    # Top-left phase badge
    draw.rounded_rectangle([24, 22, 430, 74], radius=10, fill=(15, 20, 30, 210))
    draw.rounded_rectangle([36, 36, 52, 52], radius=8, fill=(*accent, 255))
    draw.text((64, 34), phase_badge, fill=(248, 250, 252, 255), font=get_font(22, bold=True))

    # Step pill right below badge
    draw.rounded_rectangle([24, 84, 215, 118], radius=8, fill=(15, 20, 30, 180))
    draw.text((38, 90), f"STATE: {step_label}", fill=(186, 204, 224, 255), font=get_font(16, bold=True))

    # Top-right PiP Wrist Camera
    pw, ph = 288, 180
    px, py = W - pw - 24, 22
    draw.rounded_rectangle([px - 6, py - 6, px + pw + 6, py + ph + 32], radius=12, fill=(15, 20, 30, 215))
    draw.text((px + 6, py + ph + 6), "WRIST CAM  (20x20 mm · SAME VIEWPOINT)", fill=(186, 204, 224, 255), font=get_font(13, bold=True))

    # Bottom-left telemetry bar
    bx, by, bw, bh = 24, H - 118, 410, 94
    draw.rounded_rectangle([bx, by, bx + bw, by + bh], radius=12, fill=(15, 20, 30, 210))
    draw.text((bx + 16, by + 12), f"STS3215 GRIPPER ANGLE (q):  {q_deg:5.1f} deg", fill=(248, 250, 252, 255), font=get_font(17, bold=True))
    # Progress bar (-5 to 60 deg)
    frac = float(np.clip((q_deg - (-5.0)) / 65.0, 0.0, 1.0))
    bar_x0, bar_x1, bar_y0, bar_y1 = bx + 16, bx + bw - 16, by + 44, by + 58
    draw.rounded_rectangle([bar_x0, bar_y0, bar_x1, bar_y1], radius=6, fill=(51, 65, 85, 255))
    draw.rounded_rectangle([bar_x0, bar_y0, int(bar_x0 + (bar_x1 - bar_x0) * frac), bar_y1], radius=6, fill=(*accent, 255))
    info_txt = "  |  ".join(extra_lines) if extra_lines else "CLOSED (-5 deg)  <--->  OPEN (60 deg)"
    draw.text((bx + 16, by + 66), info_txt, fill=(148, 163, 184, 255), font=get_font(13, bold=False))

    out = Image.alpha_composite(img, overlay).convert("RGB")
    pip_img = Image.fromarray(pip_rgb).resize((pw, ph), Image.Resampling.LANCZOS)
    out.paste(pip_img, (px, py))
    return np.array(out)


# ---------------------------------------------------------------------------
# 6. Solve Sourccey 5-DoF IK for Scene 2
# ---------------------------------------------------------------------------
def solve_robot_trajectory(traj):
    arm = SourcceyArm(str(ARM_DIR / "ArmLeft.urdf"))
    # World pose of robot_mount: pos=(-0.10, -0.299, 0.34), euler=(0, 0, +pi/2)
    T_world_base = np.eye(4)
    T_world_base[:3, :3] = Rot.from_euler("z", math.pi / 2).as_matrix()
    T_world_base[:3, 3] = [-0.10, -0.299, 0.34]
    T_base_world = np.linalg.inv(T_world_base)

    # Initial guess: elbow-UP configuration above the table
    q_prev = np.array([0.0, 0.65, -1.05, 0.40, 0.0])
    Qs = []
    errs = []
    for pos, q_deg, _, _, _ in traj:
        # Target tool pose in world: jaws pointing radially outward from shoulder (-0.34, 0)
        yaw = math.atan2(pos[1] - 0.0, pos[0] - (-0.34))
        T_world_tool = np.eye(4)
        T_world_tool[:3, :3] = Rot.from_euler("z", yaw).as_matrix()
        T_world_tool[:3, 3] = pos
        T_target_base = T_base_world @ T_world_tool
        q_sol, ep, er = arm.ik(T_target_base, q_prev, w_ori=0.05, bounds=(-1.55, 1.55))
        q_prev = q_sol
        Qs.append((q_sol, q_deg, ep))
        errs.append(ep)
    print(f"Robot IK solved {len(Qs)} frames: mean err = {np.mean(errs):.2f} mm, max err = {np.max(errs):.2f} mm")
    return Qs


# ---------------------------------------------------------------------------
# 7. Renderers for Scene 1 & Scene 2
# ---------------------------------------------------------------------------
def render_umi_frames(traj, indices=None):
    xml = build_umi_scene_xml()
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    r_main = mujoco.Renderer(m, 720, 1280)
    r_pip = mujoco.Renderer(m, 360, 576)

    id_umi = m.body("umi_root").mocapid[0]
    id_blk = m.body("block").mocapid[0]
    id_spr = m.body("spring_body").mocapid[0]
    id_spr_geom = m.geom("spring_geom").id

    j_q = m.joint("umi_q").qposadr[0]
    j_i1 = m.joint("idx_j1").qposadr[0]
    j_i2 = m.joint("idx_j2").qposadr[0]
    j_i3 = m.joint("idx_j3").qposadr[0]

    frames = []
    idxs = range(len(traj)) if indices is None else indices
    for idx in idxs:
        pos, q_deg, bpos, label, carry = traj[idx]
        q_rad = math.radians(q_deg)

        # UMI pose (slight radial yaw matching natural human wrist)
        yaw = math.atan2(pos[1], pos[0] - (-0.34))
        R_u = Rot.from_euler("z", yaw)
        quat_xyzw = R_u.as_quat()
        d.mocap_pos[id_umi] = pos
        d.mocap_quat[id_umi] = [quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]]
        d.mocap_pos[id_blk] = bpos
        d.mocap_quat[id_blk] = [1, 0, 0, 0]

        # Gripper hinge & index finger trigger curl
        d.qpos[j_q] = q_rad
        pull = np.clip((56.0 - q_deg) / 61.0, 0.0, 1.0)
        d.qpos[j_i1] = 0.08 + 0.32 * pull
        d.qpos[j_i2] = 0.12 + 0.45 * pull
        d.qpos[j_i3] = 0.10 + 0.35 * pull

        # Dynamic tension spring between anchor (-0.042, -0.0353, -0.026) and rotating lever tail (-0.016, -0.0353, +0.024)
        p_anc_l = np.array([-0.042, -0.0353, -0.026])
        c, s = math.cos(-q_rad), math.sin(-q_rad)
        tail_x = -0.016 * c + 0.024 * s
        tail_z = 0.016 * s + 0.024 * c
        p_tail_l = np.array([tail_x, -0.0353, tail_z])
        p_anc_w = pos + R_u.apply(p_anc_l)
        p_tail_w = pos + R_u.apply(p_tail_l)
        mid = 0.5 * (p_anc_w + p_tail_w)
        vec = p_tail_w - p_anc_w
        length = float(np.linalg.norm(vec))
        m.geom_size[id_spr_geom, 1] = max(0.005, length * 0.5 - 0.002)
        T_al = trimesh.geometry.align_vectors([0, 0, 1], vec / max(length, 1e-6))
        q_sp = Rot.from_matrix(T_al[:3, :3]).as_quat()
        d.mocap_pos[id_spr] = mid
        d.mocap_quat[id_spr] = [q_sp[3], q_sp[0], q_sp[1], q_sp[2]]

        mujoco.mj_forward(m, d)
        r_main.update_scene(d, camera="main_cam")
        main_rgb = r_main.render()
        r_pip.update_scene(d, camera="wrist_cam")
        pip_rgb = r_pip.render()

        hud = composite_hud(
            main_rgb, pip_rgb,
            phase_badge="PHASE 1 · HUMAN UMI DEMO",
            accent=(34, 197, 94),
            step_label=label,
            q_deg=q_deg,
            extra_lines=(f"SPRING: {length*1000:.0f} mm", "TORQUE: OFF (READ-ONLY)"),
        )
        frames.append(hud)

    r_main.close()
    r_pip.close()
    return frames


def render_robot_frames(traj, q_sols, indices=None):
    xml = build_robot_scene_xml()
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    r_main = mujoco.Renderer(m, 720, 1280)
    r_pip = mujoco.Renderer(m, 360, 576)

    id_blk = m.body("block").mocapid[0]
    j_ids = [m.joint(n).qposadr[0] for n in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")]

    frames = []
    idxs = range(len(traj)) if indices is None else indices
    for idx in idxs:
        pos, q_deg, bpos, label, carry = traj[idx]
        q_arm, _, err_mm = q_sols[idx]
        for k in range(5):
            d.qpos[j_ids[k]] = q_arm[k]
        # In ArmLeft.urdf, gripper joint axis is -x in Gripper-Base frame, positive opens finger
        d.qpos[j_ids[5]] = math.radians(q_deg)

        mujoco.mj_forward(m, d)
        # Keep block synced with robot gripper if carrying, else on table
        if carry:
            # Place block relative to actual tool_frame so grasp looks 100% locked
            tid = m.body("tool_frame").id
            tpos = d.xpos[tid]
            tmat = d.xmat[tid].reshape(3, 3)
            d.mocap_pos[id_blk] = tpos + tmat @ (-GRASP_OFFSET)
            q_b = Rot.from_matrix(tmat).as_quat()
            d.mocap_quat[id_blk] = [q_b[3], q_b[0], q_b[1], q_b[2]]
        else:
            d.mocap_pos[id_blk] = bpos
            d.mocap_quat[id_blk] = [1, 0, 0, 0]
        mujoco.mj_forward(m, d)

        r_main.update_scene(d, camera="main_cam")
        main_rgb = r_main.render()
        r_pip.update_scene(d, camera="wrist_cam")
        pip_rgb = r_pip.render()

        j_str = " ".join(f"J{k+1}:{math.degrees(q_arm[k]):+.0f}" for k in range(5))
        hud = composite_hud(
            main_rgb, pip_rgb,
            phase_badge="PHASE 2 · SOURCCEY ROBOT",
            accent=(59, 130, 246),
            step_label=label,
            q_deg=q_deg,
            extra_lines=(j_str, f"IK ERR: {err_mm:.1f} mm"),
        )
        frames.append(hud)

    r_main.close()
    r_pip.close()
    return frames


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--preview", action="store_true", help="Render 2x2 keyframe sheet only")
    args = parser.parse_args()

    ensure_assets()
    n_scene = 180  # 6.0 seconds at 30 fps per phase
    traj = sample_task_trajectory(n_scene)
    q_sols = solve_robot_trajectory(traj)

    if args.preview:
        idxs = [20, 72, 115, 165]  # approach, grasp, transport, release
        f_umi = render_umi_frames(traj, indices=[72, 115])
        f_rob = render_robot_frames(traj, q_sols, indices=[72, 115])
        grid = np.vstack([
            np.hstack([cv2.resize(f_umi[0], (640, 360)), cv2.resize(f_umi[1], (640, 360))]),
            np.hstack([cv2.resize(f_rob[0], (640, 360)), cv2.resize(f_rob[1], (640, 360))]),
        ])
        out = CACHE_DIR / "demo_preview.png"
        Image.fromarray(grid).save(out)
        print("Saved preview to", out)
        return

    print("Rendering Phase 1 (Human UMI)...")
    f_umi = render_umi_frames(traj)
    print("Rendering Phase 2 (Sourccey Robot)...")
    f_rob = render_robot_frames(traj, q_sols)

    title_card = make_card(
        1280, 720,
        "SOURCCEY UMI",
        "Handheld Data Collection with the Real Sourccey Gripper",
        tag="VULCAN ROBOTICS · OPEN-SOURCE HARDWARE",
        accent=(34, 197, 94),
    )
    mid_card = make_card(
        1280, 720,
        "ZERO-SHOT TRANSFER",
        "Recorded STS3215 Gripper Angle + 5-DoF IK  ->  Real Sourccey Arm",
        tag="SAME GRIPPER · SAME CAMERA VIEWPOINT",
        accent=(59, 130, 246),
    )
    outro_card = make_card(
        1280, 720,
        "github.com/meisaaczac/sourccey-umi",
        "CAD  ·  Assembly Guide  ·  STS3215 Reader  ·  IK Pipeline",
        tag="READY TO BUILD",
        accent=(249, 115, 22),
    )

    all_frames = (
        [title_card] * 36
        + f_umi
        + [mid_card] * 36
        + f_rob
        + [outro_card] * 36
    )

    # Write MP4
    mp4_path = MEDIA_DIR / "sourccey_umi_demo.mp4"
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    vw = cv2.VideoWriter(str(mp4_path), fourcc, 30.0, (1280, 720))
    for fr in all_frames:
        vw.write(cv2.cvtColor(fr, cv2.COLOR_RGB2BGR))
    vw.release()
    print(f"Wrote {mp4_path} ({mp4_path.stat().st_size / 1e6:.2f} MB, {len(all_frames)} frames)")

    # Write compact looping GIF for GitHub README (skip title/outro cards so GIF jumps straight into action)
    gif_frames = []
    action_frames = f_umi[::3] + [mid_card] * 8 + f_rob[::3]
    for fr in action_frames:
        gif_frames.append(Image.fromarray(fr).resize((640, 360), Image.Resampling.LANCZOS).quantize(colors=128, method=Image.Quantize.FASTOCTREE))
    gif_path = MEDIA_DIR / "sourccey_umi_demo.gif"
    gif_frames[0].save(
        gif_path,
        save_all=True,
        append_images=gif_frames[1:],
        duration=100,
        loop=0,
        optimize=True,
    )
    print(f"Wrote {gif_path} ({gif_path.stat().st_size / 1e6:.2f} MB, {len(gif_frames)} frames)")


if __name__ == "__main__":
    main()
