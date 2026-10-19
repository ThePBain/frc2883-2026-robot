// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you may modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.aim;

/**
 * Result of the aim calculation: where the turret wants to point, whether that's in a
 * physical dead zone, and what robot heading would put the turret on target (for drive-to-aim).
 * Pose comes from swerve odometry (fused with Limelight when it sees any AprilTag); target
 * from Locations (hub/shuttle). No need to see the target—we always know where to aim.
 */
public final class AimResult {

    /** Desired turret angle (robot-relative, deg). May be clamped to allowed range if in dead zone. */
    public final double turretDeg;

    /** Raw desired turret angle before dead-zone clamping (robot-relative, deg). */
    public final double turretDegRaw;

    /** True if the ideal turret angle lies in a physical dead zone (shooter cannot point there). */
    public final boolean inDeadZone;

    /**
     * Field-relative heading (deg, 0 = +X, 90 = +Y in WPILib) the robot should have so that
     * the turret can point at the target (e.g. turret at 0°). Use for auto-aim drive / driver feedback.
     */
    public final double desiredRobotHeadingDeg;

    /** True if we can shoot: turret in allowed range and not in dead zone. */
    public final boolean canShoot;

    /** Horizontal distance to target (m). */
    public final double distanceMeters;

    public AimResult(
            double turretDeg,
            double turretDegRaw,
            boolean inDeadZone,
            double desiredRobotHeadingDeg,
            boolean canShoot,
            double distanceMeters) {
        this.turretDeg = turretDeg;
        this.turretDegRaw = turretDegRaw;
        this.inDeadZone = inDeadZone;
        this.desiredRobotHeadingDeg = desiredRobotHeadingDeg;
        this.canShoot = canShoot;
        this.distanceMeters = distanceMeters;
    }
}
