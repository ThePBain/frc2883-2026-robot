// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Turret;

/**
 * Computes turret, hood, and flywheel setpoints for an aimed shot from robot pose and field target.
 * Uses WPILib InterpolatingDoubleTreeMap for distance → RPM and distance → hood (linear interpolation).
 */
public class ShotCalculator {

  /** WPILib interpolation: distance (m) → flywheel RPM. Handles interpolation and out-of-range. */
  private static final InterpolatingDoubleTreeMap FLYWHEEL_TABLE = InterpolatingDoubleTreeMap.ofEntries(
      Map.entry(1.5, 2616.0),
      Map.entry(1.75, 2724.0),
      Map.entry(2.0, 2724.0),
      Map.entry(2.25, 2724.0),
      Map.entry(2.5, 2726.0),
      Map.entry(2.75, 2832.0),
      Map.entry(3.0, 2872.0),
      Map.entry(3.25, 3067.0),
      Map.entry(3.5, 3187.0),
      Map.entry(3.75, 3305.0),
      Map.entry(4.0, 3405.0),
      Map.entry(4.25, 3505.0),
      Map.entry(4.5, 3605.0),
      Map.entry(4.75, 3730.0));

    /** WPILib interpolation: distance (m) → flywheel RPM. Handles interpolation and out-of-range. */
  private static final InterpolatingDoubleTreeMap FLYWHEEL_TABLE_MAP_VALUES = InterpolatingDoubleTreeMap.ofEntries(
      Map.entry(1.5, 1.5),
      Map.entry(1.75, 1.75),
      Map.entry(2.0, 2.0),
      Map.entry(2.25, 2.25),
      Map.entry(2.5, 2.5),
      Map.entry(2.75, 2.75),
      Map.entry(3.0, 3.0),
      Map.entry(3.25, 3.25),
      Map.entry(3.5, 3.50),
      Map.entry(3.75, 3.750),
      Map.entry(4.0, 4.0),
      Map.entry(4.25, 4.250),
      Map.entry(4.5, 4.50),
      Map.entry(4.75, 4.75));


  /** WPILib interpolation: distance (m) → hood motor rotations. */
  private static final InterpolatingDoubleTreeMap HOOD_TABLE = InterpolatingDoubleTreeMap.ofEntries(
      Map.entry(1.5, 0.0),
      Map.entry(1.75, 0.0),
      Map.entry(2.0, 0.01),
      Map.entry(2.25, 0.222),
      Map.entry(2.5, 0.385),
      Map.entry(2.75, 0.470),
      Map.entry(3.0, 0.600),
      Map.entry(3.25, 0.600),
      Map.entry(3.5, 0.675),
      Map.entry(3.75, 0.680),
      Map.entry(4.0, .760),
      Map.entry(4.7, .970));

  // ---- Shuttle tables: different RPM/hood for lob shots across the field ----

  /** Shuttle: distance (m) → flywheel RPM. Tune these for your shuttle shot. */
  private static final InterpolatingDoubleTreeMap SHUTTLE_FLYWHEEL_TABLE = InterpolatingDoubleTreeMap.ofEntries(
            Map.entry(4.0, 2500.0),
      Map.entry(5.0, 2725.0),
      Map.entry(6.0, 2950.0),
      Map.entry(7.0, 3175.0),
      Map.entry(8.0, 3400.0),
      Map.entry(9.0, 3625.0),
      Map.entry(10.0, 3850.0),
      Map.entry(11.0, 4075.0),
      Map.entry(12.0, 4300.0),
      Map.entry(13.0, 4525.0),
      Map.entry(14.0, 4750.0),
      Map.entry(15.0, 4975.0),
      Map.entry(16.0, 5200.0),
      Map.entry(17.0, 5425.0));

  /** Shuttle: distance (m) → flywheel RPM key mirror (for custom interpolation). */
  private static final InterpolatingDoubleTreeMap SHUTTLE_FLYWHEEL_MAP_VALUES = InterpolatingDoubleTreeMap.ofEntries(
      Map.entry(4.0, 4.0),
      Map.entry(5.0, 5.0),
      Map.entry(6.0, 6.0),
      Map.entry(7.0, 7.0),
      Map.entry(8.0, 8.0),
      Map.entry(9.0, 9.0),
      Map.entry(10.0, 10.0),
      Map.entry(11.0, 5.0),
      Map.entry(12.0, 6.0),
      Map.entry(13.0, 7.0),
      Map.entry(14.0, 8.0),
      Map.entry(15.0, 9.0),
      Map.entry(16.0, 8.0),
      Map.entry(17.0, 9.0));

  /** Shuttle: distance (m) → hood motor rotations. Tune these for your shuttle shot. */
  private static final InterpolatingDoubleTreeMap SHUTTLE_HOOD_TABLE = InterpolatingDoubleTreeMap.ofEntries(
    Map.entry(2.0, 1.5),
      Map.entry(3.0, 1.5),
      Map.entry(4.0, 1.5),
      Map.entry(5.0, 1.5),
      Map.entry(6.0, 1.5),
      Map.entry(7.0, 1.5),
      Map.entry(8.0, 1.5),
      Map.entry(9.0, 1.5),
      Map.entry(10.0, 1.5),
      Map.entry(11.0, 1.5),
      Map.entry(12.0, 1.5),
      Map.entry(13.0, 1.5),
      Map.entry(14.0, 1.5),
      Map.entry(15.0, 1.5),
      Map.entry(16.0, 1.5),
      Map.entry(17.0, 1.5));
      

  /** Result of a shot calculation: setpoints for turret, hood, and flywheel. */
  public static final class ShotSetpoints {
    public double turretRotations;
    public double hoodPosition;
    public double flywheelRpm;
    /** True if setpoints are valid. */
    public boolean valid;
    /** Horizontal distance to target (m). */
    public double distanceMeters;
  }

  private final Field2d m_aimField = new Field2d();
  private final FieldObject2d m_turretObj = m_aimField.getObject("Turret");

  public ShotCalculator() {
    SmartDashboard.putData("Aim/Field", m_aimField);
  }

  /**
   * Pose-based aim: from robot pose and target position on field, compute turret/hood/flywheel.
   * Use this when Limelight(s) have corrected pose (we don't need to see the hub—any AprilTag helps).
   *
   * @param robotPose    Current robot pose (from drivetrain.getState().Pose).
   * @param targetX      Target X on field (m).
   * @param targetY      Target Y on field (m).
   * @param targetZ      Target height above field (m); used for hood.
   * @return Setpoints; valid is true if distance &gt; 0.1 m.
   */
  public ShotSetpoints getSetpointsFromPose(CommandSwerveDrivetrain robotDrive, Turret turret, double targetX, double targetY, double targetZ) {
    return getSetpointsFromPose(robotDrive, turret, targetX, targetY, targetZ, false);
  }

  public ShotSetpoints getSetpointsFromPose(CommandSwerveDrivetrain robotDrive, Turret turret, double targetX, double targetY, double targetZ, boolean shuttle) {
    ShotSetpoints out = new ShotSetpoints();
    var robotState = robotDrive.getState();
    var robotPose = robotState.Pose;
    // Turret offset is in robot frame (X=forward, Y=left). Rotate to field frame.
    Translation2d turretOffsetRobot = new Translation2d(
        Constants.ShotCalculator.kTurretOffsetX_M,
        Constants.ShotCalculator.kTurretOffsetY_M);
    Translation2d turretOffsetField = turretOffsetRobot.rotateBy(robotPose.getRotation());
    ChassisSpeeds speeds = robotState.Speeds;
    double headingRad = robotPose.getRotation().getRadians();
    // Turret lead: predict robot pose when turret settles, aim at hub from that future pose
    double cosH = Math.cos(headingRad);
    double sinH = Math.sin(headingRad);
    double fieldVx = speeds.vxMetersPerSecond * cosH - speeds.vyMetersPerSecond * sinH;
    double fieldVy = speeds.vxMetersPerSecond * sinH + speeds.vyMetersPerSecond * cosH;
    double leadT = Constants.ShotCalculator.kTurretLeadTimeSeconds;
    double xFuture = robotPose.getX() + fieldVx * leadT;
    double yFuture = robotPose.getY() + fieldVy * leadT;
    double turretX = xFuture + turretOffsetField.getX();
    double turretY = yFuture + turretOffsetField.getY();
    double dx = targetX - turretX;
    double dy = targetY - turretY;
    double dz = targetZ - Constants.ShotCalculator.kTurretOffsetZ_M;
    double distanceM = Math.sqrt(dx*dx + dy*dy + dz*dz);
    double groundDistanceM = Math.hypot(dx, dy);
    out.distanceMeters = groundDistanceM;

    if (groundDistanceM < 0.1) {
      out.valid = false;
      return out;
    }
    out.valid = true;

    double headingFutureRad = headingRad + speeds.omegaRadiansPerSecond * leadT;
    // Turret offset at future pose: rotate robot-frame offset by future heading
    Translation2d turretOffsetFuture = turretOffsetRobot.rotateBy(
        Rotation2d.fromRadians(headingFutureRad));
    double turretXFuture = xFuture + turretOffsetFuture.getX();
    double turretYFuture = yFuture + turretOffsetFuture.getY();
    double dxLead = targetX - turretXFuture;
    double dyLead = targetY - turretYFuture;
    double fieldAngleToTargetDeg = Math.toDegrees(Math.atan2(dyLead, dxLead));
    double turretHeadingDeg = Math.toDegrees(headingFutureRad) + Constants.ShotCalculator.kTurretOffsetYawDeg;
    double turretAngleDeg = MathUtil.inputModulus(fieldAngleToTargetDeg - turretHeadingDeg, -180.0, 180.0);
    SmartDashboard.putNumber("Aim/TurretLeadAngleDeg", turretAngleDeg);
    m_turretObj.setPose(new Pose2d(turretXFuture, turretYFuture, Rotation2d.fromDegrees(turretHeadingDeg)));

    // Velocity compensation: decompose field velocity into radial (toward/away from hub) and tangential.
    double unitDx = dx / groundDistanceM;
    double unitDy = dy / groundDistanceM;
    double velocityTowardHubMps = fieldVx * unitDx + fieldVy * unitDy;    // positive = toward hub
    double velocityTangentialMps = -fieldVx * unitDy + fieldVy * unitDx;  // positive = CCW around hub
    SmartDashboard.putNumber("Velocity towards target",velocityTowardHubMps);
    SmartDashboard.putNumber("Velocity about target", velocityTangentialMps);

    // Tangential compensation: atan(vTangential / muzzleSpeed) gives the physics-based deflection angle.
    // Non-linear: small at low speed, saturates at high speed, grounded in real ball trajectory.
    double tangentialOffsetDeg = -Math.toDegrees(
        Math.atan2(velocityTangentialMps, Constants.ShotCalculator.kNominalMuzzleSpeedMps))
        * Constants.ShotCalculator.kTangentialCompensationScale;
    turretAngleDeg += tangentialOffsetDeg;
    turretAngleDeg = MathUtil.inputModulus(turretAngleDeg, -180.0, 180.0);

    // Turret setpoint: angle (deg) → motor rotations (after tangential compensation)
    double zero = Constants.TurretKinematics.kZeroPositionRotations;
    double curPosition = turret.getPosition();
    double degPerRot = Constants.TurretKinematics.kDegreesPerMotorRotation;
    out.turretRotations = zero + (turretAngleDeg / degPerRot);
    if (out.turretRotations >= Constants.TurretLimits.kMaxPositionRotations
        || out.turretRotations <= Constants.TurretLimits.kMinPositionRotations) {
      out.valid = false;
    }
    SmartDashboard.putNumber("Rotation Target - Turret Rotation", out.turretRotations - curPosition);

    // Pick tables: shuttle uses separate RPM/hood curves
    var flywheelTable = shuttle ? SHUTTLE_FLYWHEEL_TABLE : FLYWHEEL_TABLE;
    var flywheelMapValues = shuttle ? SHUTTLE_FLYWHEEL_MAP_VALUES : FLYWHEEL_TABLE_MAP_VALUES;
    var hoodTable = shuttle ? SHUTTLE_HOOD_TABLE : HOOD_TABLE;

    var LowerBoundGround = groundDistanceM - 0.25;
    var UpperBoundGround = groundDistanceM + 0.25;
    var LowRPM = flywheelTable.get(LowerBoundGround);
    var HighRPM = flywheelTable.get(UpperBoundGround);
    var LowRPMKey = flywheelMapValues.get(LowerBoundGround);
    var HighRMPKey = flywheelMapValues.get(UpperBoundGround);

    out.hoodPosition = hoodTable.get(groundDistanceM);
    if((yFuture > 4.0 && yFuture < 5.0 )|| (yFuture > 11.5 && yFuture < 12.5)){ //Hood safety
      out.hoodPosition = 0;
    }
    else
    {
      // Radial hood compensation: moving toward hub → lower hood (flatter), away → raise hood (steeper).
      double hoodComp = -Constants.ShotCalculator.kHoodCompensationRotPerMpsTowardHub * velocityTowardHubMps;
      out.hoodPosition += hoodComp;
      SmartDashboard.putNumber("Aim/HoodCompensation", hoodComp);
    }
    var RealRPM = LowRPM + ( ((groundDistanceM - LowRPMKey)/(HighRMPKey - LowRPMKey)) * (HighRPM - LowRPM));
    out.flywheelRpm = RealRPM;

    // Radial RPM compensation: moving toward hub → subtract RPM, away → add RPM.
    out.flywheelRpm -= Constants.ShotCalculator.kVelocityCompensationRpmPerMpsTowardHub * velocityTowardHubMps;

    SmartDashboard.putNumber("Aim/VelTowardHub_mps", velocityTowardHubMps);
    SmartDashboard.putNumber("Aim/VelTangential_mps", velocityTangentialMps);
    SmartDashboard.putNumber("Aim/TangentialOffsetDeg", tangentialOffsetDeg);
    SmartDashboard.putNumber("Aim/FieldVx", fieldVx);
    SmartDashboard.putNumber("Aim/FieldVy", fieldVy);
    SmartDashboard.putNumber("Aim/RpmCompensation",
        -Constants.ShotCalculator.kVelocityCompensationRpmPerMpsTowardHub * velocityTowardHubMps);

    out.flywheelRpm = MathUtil.clamp(out.flywheelRpm,
        Constants.FlywheelVelocity.MIN_RPM,
        Constants.FlywheelVelocity.MAX_RPM);
    out.hoodPosition = MathUtil.clamp(out.hoodPosition,
        Constants.HoodLimits.kMinPosition,
        Constants.HoodLimits.kMaxPosition);

    return out;
  }
}
