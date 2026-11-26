# Data collection (suggested protocol)

> These are my suggestions, not validated in the field.

1. Calibrate the servo and check the travel (65° ± 5°).
2. Start each episode with 3 slow open/close cycles in view of the tracker (marker-based sync).
3. Keep the gripper **open between grasps** so the data stays clean.
4. **Point "radially" at the robot**: the Sourccey arm has 5 joints and cannot do an arbitrary hand yaw. Turn your body, not your wrist, and avoid yaw rotations outside the arm's plane.
5. Smooth motions (the quality filter rejects anything above 2 m/s).
6. After each session: `build_episode.py` and look at `valid_pct` (goal > 80%) and `err_pos_p95` (< 15 mm).
7. Replay at low speed on the robot or in the Sourccey MuJoCo sim before training.

Episode format (`episode.npz`): `t, arm_q[N,5], gripper_q_deg, tool_pose[N,4,4], err_pos_mm, err_ori_deg, speed, valid, lost`.
