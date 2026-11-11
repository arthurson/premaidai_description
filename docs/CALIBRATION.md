# Calibration

The model assumes the factory calibration of a PreMaid AI: every servo at raw `7500` is at joint angle 0 (the T-pose of the simulator),
`angle = sign × (raw − 7500) × 270 / 8000` degrees (see PROTOCOL.md). The 25 servo directions (`sign`) have been verified on a real robot.
If your robot disagrees with the simulator, calibrate in this order.

1. **Centre.** Put the robot on its stand, open the *Robot* tab, press *T 待機 / T pose* and compare it with the simulator's `t_pose` keyframe.
   An arm or leg that is visibly off means the horn is on the wrong spline tooth (mechanical fix) or the servo trim is not zero.
2. **Trim.** The 14-byte servo state (PROTOCOL.md 3.1) contains the trim. Enable *Poll servo status* and read the `trim` / `actual` columns; the custom firmware keeps
   per-servo offsets in `ServoConfig.h` (`homePosition`). Change one servo at a time and re-check the pose.
3. **Direction.** With *Twin* on and the servos released, move a limb by hand: the simulator must follow in the same direction (this needs firmware that reads positions back).
   Without read-back, move the sliders of one joint with *Live control* on and compare. Fix a wrong sign in the firmware table, not by re-wiring the model.
4. **Limits.** Leave *Safe limits* on: it clips poses to the range in the servo table (PROTOCOL.md 3.3) intersected with the limits stored in the simulator. Widen them only after checking the mechanics.
5. **Speed / stretch.** `0x19` frames set per-servo speed (1…127) and stretch (stiffness, 1…127). Defaults of premaiduino2: lower body speed 127 / stretch 60, upper body 100 / 100.
6. **Battery.** The voltage reply is `raw / 216` volts. Compare with a multimeter once and look at the battery graph (*Settings → serial monitor*): the dashed red line is 9 V.
7. **IMU.** The simulator's sensor HUD shows what a perfect IMU on the base would read (accelerometer ≈ 9.81 m/s² up at rest, gyro 0). Compare it with the robot's MPU-6050 while it stands still:
   a constant gyro offset is the bias to subtract in firmware.

Record a calibration pose as CSV: *Record tab → Teach → Capture → Export CSV* writes raw and angle of every servo per keyframe.
