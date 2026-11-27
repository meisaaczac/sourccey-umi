# Credits and third-party parts

- **Sourccey** (Vulcan Robotics, https://github.com/vulcan-forge/sourccey-hardware): the gripper base and finger STLs in `cad/stl_sourccey/` are oriented copies of the official Sourccey meshes from Vulcan Robotics' repo (reproducible via `tools/prepare_sourccey_parts.py`). Their repository does not declare an explicit license file at the time of writing; all rights to the original Sourccey parts belong to Vulcan Robotics.
- **HandUMI** (Apache-2.0): the idea of using a Feetech servo as the encoder and swappable tips per robot.
- **Grabette / Gripette** (Pollen Robotics, Apache-2.0): handheld gripper with encoders, SLAM and LeRobot export; the same idea of using the robot's own gripper by hand.
- **UMI** (Stanford, MIT) and **actuated-UMI** (MIT): handheld gripper with a wide-angle camera and SLAM.
- **HiFi-UMI** (arXiv, 2026): comparison of UMI designs (trigger vs finger sleeve) and encoder/pose requirements.
No files from those projects were copied here; they were design references.
