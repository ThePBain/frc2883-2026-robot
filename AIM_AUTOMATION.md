# Aim automation (turret + dead zones + desired robot heading)

Aim is **fully automatic** from pose + target: you do **not** need to see an AprilTag or the hub to know where to aim. Pose comes from swerve odometry (Limelight corrects it when it sees *any* tag); target comes from **Locations** (hub / shuttle). Together they give turret angle, hood, and flywheel.

---

## How it works

1. **Pose** — Drivetrain keeps robot pose (x, y, heading). Limelight, when it sees any AprilTag, pushes a vision measurement so odometry stays accurate. Advantage Scope can log this same pose.
2. **Target** — You pick hub or shuttle via `RobotState.setSelectedTargetName("HUB_BLUE")` (or HUB_RED, SHUTTLE_*, etc.). Coordinates are in **Locations** (field drawings).
3. **Aim** — Each cycle we compute:
   - **Turret angle** (robot-relative) to the target, with velocity compensation (shoot on the move).
   - **Dead zone** — If that angle is outside the allowed turret arc (e.g. shooter would hit the frame), we clamp turret to the nearest safe angle and set **canShoot = false**.
   - **Desired robot heading** — Field angle (deg) the robot should have so the turret could point at the target (e.g. turret at 0°). Used for auto-aim drive and driver feedback.

4. **Shoot** — `ShootCommand` only feeds when **isReadyToShoot()** is true, which requires **canShoot** (turret not in dead zone) plus flywheel/hood/turret at setpoint.

---

## Turret dead zones

- **Constants.Turret**: `kAllowedMinDeg`, `kAllowedMaxDeg` define the **allowed** arc (robot-relative). Example: **-90 to 90** = turret can only point “forward” (left 90° to right 90°). Outside that is a dead zone.
- When the ideal turret angle is in a dead zone:
  - Turret is commanded to the **nearest allowed angle** (so it doesn’t drive into the frame).
  - **CanShoot** is false → **isReadyToShoot()** is false → no feed.
  - Driver can use **auto-aim (X)** to rotate the robot to **DesiredRobotHeadingDeg**; then the turret can point at the target and **CanShoot** becomes true.

---

## SmartDashboard (and Advantage Scope)

When preparing to shoot (left/right trigger), you get:

| Key | Meaning |
|-----|--------|
| **Shooter/TurretDeg** | Current turret angle (encoder). |
| **Shooter/TurretTargetDeg** | Commanded angle (clamped to allowed range if in dead zone). |
| **Shooter/TurretDegRaw** | Ideal turret angle before dead-zone clamp. |
| **Shooter/InDeadZone** | True if ideal angle is in a forbidden arc. |
| **Shooter/CanShoot** | True if turret is in allowed range (can shoot when at setpoint). |
| **Shooter/DesiredRobotHeadingDeg** | Field heading (deg) to rotate to so turret can aim (for auto-aim / driver). |
| **Shooter/DistanceM** | Horizontal distance to target (m). |

Log these in Advantage Scope to debug pose, dead zones, and shot readiness.

---

## What you need to set

1. **Locations** — Hub and shuttle (x, y, z) from field drawings in `constants/Locations.java`.
2. **Turret allowed arc** — In **Constants.Turret**: `kAllowedMinDeg`, `kAllowedMaxDeg` to match your mechanical limits (e.g. -90, 90).
3. **Pose** — Odometry is automatic; Limelight keeps it corrected when it sees tags. No need to see the hub to aim.

---

## Summary

- **Aim** = pose (odometry + Limelight) + target (Locations) → turret/hood/flywheel.
- **Dead zone** = turret can’t point there; we clamp to nearest safe angle and set **CanShoot** false until the driver rotates (e.g. with auto-aim).
- **Desired robot heading** = where to point the drive base so the turret can aim; same value auto-aim drive uses when you press X.
