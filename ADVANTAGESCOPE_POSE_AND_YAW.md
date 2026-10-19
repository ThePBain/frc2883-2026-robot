# AdvantageScope: Robot Pose and Yaw Issues

This doc summarizes causes of wrong pose (especially yaw) in AdvantageScope and how they were fixed or worked around.

---

## 1. Correct in one view, “ballistic” yaw when tracking on 2D Field

**Symptom:** Pose looks correct in the data tree / raw view, but when you **drag the robot pose into the 2D Field “Poses” section** to track the robot, the **yaw goes wrong** (wild rotation).

**Cause:** We publish pose in two ways:

- **Struct:** `DriveState/Pose` and (now) `Pose/Robot` — byte-encoded `Pose2d` (rotation in radians inside the struct). AdvantageScope interprets these correctly.
- **Legacy array:** `Pose/robotPose` — double array `[x, y, rotation]`. AdvantageScope’s docs say the legacy array format requires **manual specification of packing and units**. If the 2D Field treats the third element as **radians** (common default) but we send **degrees**, then e.g. -152 is interpreted as -152 rad and the drawn yaw is wrong.

**Fix:** Use a **struct** topic for the 2D Field robot pose:

- In AdvantageScope, for the 2D Field’s **Poses** (robot tracking), use **`DriveState/Pose`** or **`Pose/Robot`** (struct), **not** `Pose/robotPose` (array).
- We now publish `Pose/Robot` as a `Pose2d` struct so you have a dedicated struct pose under the Pose table.

---

## 2. Yaw drifts when robot is not moving or is disabled

**Symptom:** Reported yaw (or full pose) keeps changing even when the robot is stationary or disabled.

**Possible causes:**

- **Gyro drift:** The Pigeon (or other IMU) can slowly drift when stationary; the pose estimator keeps fusing that, so the pose (especially heading) drifts.
- **Vision when disabled:** If the robot is disabled but the pose estimator and vision pipeline still run, Limelight updates can still be applied and can conflict with the last known pose or gyro, causing small jumps or drift.
- **Replay / timeline:** In AdvantageScope, scrubbing the timeline can show different timestamps; if pose is interpolated or logged at different rates, it can look like rotation is changing.

**What to check:**

- In AdvantageScope, compare **`DriveState/RawHeadingDegrees`** (gyro-only) with the rotation from **`DriveState/Pose`** (fused). If only the fused pose drifts, the issue is in vision fusion or estimator; if raw heading also drifts, it’s gyro/driver behavior.

---

## 3. Fixes already applied in code (recap)

- **Vision timestamp:** Vision measurement time uses `Utils.getCurrentTimeSeconds() - (latency/1000)` and is **not** converted with `fpgaToCurrentTime` in the drivetrain.
- **Limelight robot yaw:** Sent to `SetRobotOrientation` in **degrees** (not radians) so `getBotPose3d` is correct.
- **Vision theta std dev:** Passed in **radians** (e.g. `Math.toRadians(360)`), not 360 as degrees.
- **Invalid vision poses:** We only call `addVisionMeasurement` when `LimelightHelpers.validPoseEstimate(...)` is true.
- **Single camera for pose:** Only the left Limelight pushes pose; the right is created with pose collection disabled to avoid double vision updates and chaotic yaw.
- **Raw heading for comparison:** `DriveState/RawHeadingDegrees` is published so you can compare gyro-only vs fused pose in AdvantageScope.

---

## 4. Topic reference for AdvantageScope

| Topic | Format | Use for 2D Field? |
|-------|--------|--------------------|
| `DriveState/Pose` | Struct (`Pose2d`) | Yes — preferred |
| `Pose/Robot` | Struct (`Pose2d`) | Yes — preferred |
| `Pose/robotPose` | Legacy double array `[x, y, rotation_degrees]` | Only if you set rotation units to **degrees** in AdvantageScope; otherwise use struct. |
| `DriveState/RawHeadingDegrees` | Double (degrees) | For comparison with fused pose, not for drawing pose. |

Using **struct** topics for the 2D Field avoids unit and format confusion and matches AdvantageScope’s recommendation (byte-encoded struct for geometry).
