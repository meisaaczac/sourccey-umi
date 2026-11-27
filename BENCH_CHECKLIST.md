# Bench checklist: only start collecting data when EVERY item passes

Each item has a numeric criterion. If it fails, the right-hand column says what to change.

| # | Test | Criterion | If it fails |
|---|---|---|---|
| 1 | Print lever + grip, dry-fit WITHOUT the servo | Finger turns by hand from -5° to 60° without scraping the base or the grip; the trigger reaches its stop at ~60° | Adjust clearances in `cad/umi_cad.py` and reprint |
| 2 | Screws in the finger's 4-hole cross (-y side) | 4 M3 go in and tighten without play; the lever does not rotate relative to the finger | Use inserts/nuts on the inside or epoxy the hub |
| 3 | Rear interface (5 screws, base to grip) | Rigid grip: no play when twisting by hand | Use longer screws; check the mounting direction on your robot |
| 4 | Servo installed, torque off: force to move the trigger (kitchen scale on the pad) | less than ~10 N over the whole travel | Lower gear-ratio servo (1:147 or 1:191); if it persists, an AS5600 magnetic sensor on a free-spinning shaft |
| 5 | Spring return | Release the trigger at any point and the gripper opens by itself to the stop | The velcro strap becomes REQUIRED (index finger pushes); stronger spring; or swap the servo for an AS5600 with a free shaft |
| 6 | Calibration: `sts_angle.py calibrate` | Measured travel 65° ± 5° | Wrong horn or servo: redo the finger assembly |
| 7 | Repeatability: close 10 times on a 20 mm gauge block | standard deviation of q below 0.5° | Play in the lever or the horn |
| 8 | Latency: 3 slow open/close cycles (q 10° to 55°) in view of the Sourccey 20×20 mm wrist camera (or external camera) with the ArUco markers; run `sync_offset.py` | correlation above 0.9 and the lag repeats within ±10 ms over 5 recordings | Better lighting / bigger markers; repeat |
| 9 | Pose: move the device 200 mm along a ruler | error below 5 mm; no `lost` frames in SLAM | Calibrate `T_tracker_tool`; revisit the tracker mount |
| 10 | 1-minute episode: `record_episode.py` + SLAM + `build_episode.py` | valid above 80%; `err_pos_p95` below 15 mm | Record with the gripper pointing "radially" to the robot (see the 5-joint arm limits) |
| 11 | Replay on the robot (or in the Sourccey MuJoCo sim) at low speed | Motion follows the episode without hitting anything | Review T_base_world and the calibration |

Only then train a policy. For the operator: keep the gripper open between grasps and avoid yaw rotations that the 5-joint arm cannot make.
