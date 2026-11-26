# Credits and third-party parts

- **Sourccey** (Vulcan Robotics, https://github.com/vulcan-forge/sourccey-hardware): the base, finger and servo of this gripper belong to their robot.
  This repo does NOT include those files. `tools/prepare_sourccey_parts.py` downloads the STLs straight from the source. Vulcan's repository does not declare a license
  at the time of writing: confirm with them before redistributing the parts.
- **HandUMI** (Apache-2.0): the idea of using a Feetech servo as the encoder and swappable tips per robot.
- **Grabette / Gripette** (Pollen Robotics, Apache-2.0): handheld gripper with encoders, SLAM and LeRobot export; the same idea of using the robot's own gripper by hand.
- **UMI** (Stanford, MIT) and **actuated-UMI** (MIT): handheld gripper with a wide-angle camera and SLAM.
- **HiFi-UMI** (arXiv, 2026): comparison of UMI designs (trigger vs finger sleeve) and encoder/pose requirements.
No files from those projects were copied here; they were design references.
