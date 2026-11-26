# Research and decisions

| Project | What we took | What we did NOT copy |
|---|---|---|
| **HiFi-UMI** (arXiv 2026) | trigger vs finger-sleeve comparison, encoder < 0.1° requirement, pose quality | we use the STS3215 servo (0.088°) instead of a dedicated encoder |
| **HandUMI** (Apache-2.0) | Feetech servo as the encoder; swappable tips per robot | it is worn on the fingers; we use a trigger |
| **Grabette/Gripette** (Pollen, Apache-2.0) | the same finger on the hand and on the robot; SLAM and LeRobot | it uses magnetic sensors; its STLs are in Git LFS (blocked where I worked) |
| **UMI** and **actuated-UMI** (MIT) | handheld gripper with a wide-angle camera and SLAM | parallel jaws (the Sourccey's is hinged) |

## Decisions
- **Spring-loaded trigger** (the dominant design among the UMIs compared) rather than a finger sleeve, for automatic return and less fatigue.
- **The real Sourccey gripper**, so the measured angle is the robot's joint with no width conversion.
- **Lever on the -y side**, so the robot's camera (+y side) stays unobstructed.
- **Spring anchored inside the beam** at (x=-42, z=-26): the only region where the spring torque is opening over the whole travel (numerical sweep).
- **Side pad** (not a bar across the base): the base's fixed jaw would block a bar in the middle while opening.
- **Fallback if servo friction is high**: swap the servo for an AS5600 magnetic sensor on a free shaft (as Grabette does). Not designed yet.
- **This repo does not redistribute Vulcan's parts** (no declared license): `tools/prepare_sourccey_parts.py` downloads them from the source.

## Open points
- Which SLAM/tracker to use and how to calibrate `T_tracker_tool`.
- LeRobot export (reuse Grabette).
- Mount for the 20x20 camera (today: use your robot's).
