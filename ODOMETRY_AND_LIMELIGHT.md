# Odometry and Limelight: How It Works

## Overview

The robot uses **CTRE Phoenix 6 SwerveDrivetrain** with a built-in pose estimator that fuses:
1. **Odometry** – wheel encoders + gyro (Pigeon2), updated every odometry cycle (~250 Hz)
2. **Vision** – Limelight AprilTag pose estimates, added when a tag is visible

---

## What Each Component Does

### 1. Odometry (always running)
- Integrates wheel encoder deltas and gyro to estimate pose (x, y, θ)
- Runs on a background thread in the drivetrain
- **RawHeading** = gyro only (no vision fusion)
- **Pose** = fused estimate (odometry + vision when available)

### 2. Limelight (LL_Left only pushes pose)
- **LL_Left** has `shouldCollectPose = true` → calls `addVisionMeasurement()` when it sees a tag
- **LL_Right** has `shouldCollectPose = false` → does NOT push pose (avoids double updates)
- Vision std devs: `[0.3, 0.3, 999999]` → we trust x,y for position; we **ignore** theta so vision does not change the driver’s “forward”

### 3. Seed / resetRotation
- **seedFieldCentric()** – calls `resetRotation(gyro yaw)` to sync the pose’s θ to the Pigeon
- **resetRotation()** – updates only the pose’s rotation; does **not** clear x,y or the vision buffer
- **resetPose()** – full pose reset (used by PathPlanner when starting auto)

### 4. Left bumper (setForwardToCurrentHeading)
- Sets `m_driveHeadingOffsetDegOverride = 0` so the drive uses **RawHeading** (gyro) as “forward”
- Does **not** call resetRotation or seed
- Does **not** affect the pose estimator or Limelight

---

## Why Vision Might Seem to Stop After Seed

**resetRotation** only changes the pose’s θ. It should **not** stop vision from updating. If vision appears to stop:

1. **Timestamp** – Vision uses `Utils.getCurrentTimeSeconds() - latency`. If the time base is wrong, measurements can be rejected.
2. **Valid pose** – We only add vision when `LimelightHelpers.validPoseEstimate()` is true. Bad poses are skipped.
3. **Tag visibility** – If the robot moves so no tag is visible, vision stops until a tag is seen again.
4. **Std devs** – If theta std dev is very high (999999), we ignore vision’s θ but still use x,y. Position updates should continue.

---

## Current Flow (Simplified)

```
[Odometry thread]  wheel encoders + gyro  →  pose estimate (x, y, θ)
                         ↑
[Limelight periodic]  when tag visible: addVisionMeasurement(pose, timestamp)
                         →  Kalman filter fuses with odometry
                         →  x,y trusted (std dev 0.3 m)
                         →  θ ignored (std dev 999999) so driver heading stays from gyro

[Drive]  getFieldCentricDriveRequest() uses RawHeading + offset
         →  offset = 0 when left bumper pressed (forward = where you point)
         →  offset = alliance default when not pressed
```

---

## Things to Check If Odometry Is Off

1. **YawDebug** (SmartDashboard): Compare `PigeonYaw`, `PoseRotation`, `RawHeading`. If PoseRotation drifts from PigeonYaw, vision or fusion may be affecting θ.
2. **AdvantageScope**: Use `DriveState/Pose` (struct) for the 2D field. Compare with `DriveState/RawHeadingDegrees`.
3. **Vision theta**: We use 999999 for theta std dev so vision does not update θ. If you want vision to correct heading too, reduce it (e.g. `Math.toRadians(30)`).
4. **Single camera**: Only LL_Left pushes. If both pushed, pose could oscillate.
