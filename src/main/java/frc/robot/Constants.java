// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Location;

/** Motor IDs and test values for manual subsystem testing. */
public final class Constants {
  private Constants() {}

  /** CAN motor IDs (labeled on robot). Drive = 1–12. */
  public static final class Motors {
    public static final int INTAKE_ARM_LEFT = 14;   // Intake Arm Left x60
    public static final int INTAKE_ARM_RIGHT = 15;  // Intake Arm Right x60
    public static final int SPINDEX = 16;           // Spindex x44
    public static final int ELEVATOR = 17;         // Elevator x44
    public static final int TURRET = 18;            // Turret x44
    public static final int SHOOTER_HOOD = 19;      // Shooter Hood Kraken X44 (TalonFX)
    public static final int FLYWHEEL_LEFT = 20;     // Shooter Flywheel Left Vortex
    public static final int FLYWHEEL_RIGHT = 21;    // Shooter Flywheel Right Vortex
    public static final int INTAKE_WHEELS = 23;     // Intake Wheels x44
  }

  /** Default percent (0–1) per motor for manual testing. Start low; adjust on SmartDashboard. */
  public static final class TestSpeeds {
    public static final double INTAKE_ARM = 0.25;
    public static final double INTAKE_WHEELS = 1;
    public static final double SPINDEX = 0.4;
    public static final double ELEVATOR = 1;
    public static final double TURRET = 0.8;
    public static final double HOOD = 0.25;
    public static final double FLYWHEEL = 0.15;
  }

  public static final double FLYWHEEL_SPEED_STEP = 0.025;

  public static final double FLYWHEEL_SPEED_MIN = 0.0;
  public static final double FLYWHEEL_SPEED_MAX = 1.0;

  /** Hood travel limits (rotor rotations). 0 = lowest (flat), 1.7 = highest. Whe
   * n max < min, limits disabled. */
  public static final class HoodLimits {
    public static final double kMinPosition = -.05;    // Lowest (flat, startup)
    public static final double kMaxPosition = 1.7;  // Highest
  }

  /** Intake arm travel limits (rotor rotations of left motor). When max < min, limits are disabled. */
  public static final class IntakeArmLimits {
    public static final double kMinPositionRotations = -6;
    public static final double kMaxPositionRotations = .5;
    public static final double kHomePositionRotations = 0.0;
  }

  /** Turret travel limits (rotor rotations). When max < min, limits are disabled (safe default). */
  public static final class TurretLimits {
    public static final double kMinPositionRotations = -14;
    public static final double kMaxPositionRotations = 14;
  }

  /** Flywheel velocity (RPM) range and PID. m_speed 0–1 maps to MIN_RPM..MAX_RPM. */
  public static final class FlywheelVelocity {
    public static final double MIN_RPM = 2200.0;
    public static final double MAX_RPM = 6500.0;
    // TODO: Tune on robot. Velocity PID gains.
    public static final double kP = 0.00075;
    public static final double kI = 0;
    public static final double kD = 0.0;
    public static final double kFF = 0.000147;
  }

  /** Shot calculator: pose-based turret, hood, flywheel setpoints. Tune on robot. */
  public static final class ShotCalculator {
    /** Reference distance (m) for pose-based shot. */
    public static final double kReferenceDistanceMeters = 4.0;
    /** Flywheel RPM at reference distance. */
    public static final double kReferenceRpm = 4000;
    /** Exponent for RPM vs distance: rpm = refRpm * (d/refD)^exp. */
    public static final double kDistanceExponent = 0.3;
    /** Hood angle (deg) at reference distance. */
    public static final double kReferenceHoodAngleDeg = 25.0;
    /** Hood angle change per meter from reference distance. */
    public static final double kHoodAnglePerMeter = 1.50;
    /** Nominal muzzle speed (m/s) for time-of-flight / velocity lead. */
    public static final double kNominalMuzzleSpeedMps = 6.67;
    /** Velocity compensation: RPM to subtract per m/s of robot velocity toward the hub. Larger = more "shoot on the fly" correction. */
    public static final double kVelocityCompensationRpmPerMpsTowardHub = 475; //TODO
    /** Hood compensation: rotations to subtract per m/s of robot velocity toward the hub. Driving toward → flatten hood, away → steepen. */
    public static final double kHoodCompensationRotPerMpsTowardHub = 0.3;
    /** Tangential angle compensation: scalar on atan(vTangential / muzzleSpeed). 1.0 = pure physics, >1 = more aggressive. */
    public static final double kTangentialCompensationScale = 3;
    /** Turret lead: seconds to predict robot pose ahead (turret settle time). Larger = more aggressive lead for shooting on the move. */
    public static final double kTurretLeadTimeSeconds = 0.3;

    public static final double kTurretOffsetX_M = -0.127; // Turret center offset from robot center (m). Positive = turret forward of robot center. (5 inches) Tune on robot.
    public static final double kTurretOffsetY_M = 0.1016;   // Positive = turret left of robot center. (4 inches) Tune on robot.
    public static final double kTurretOffsetZ_M = .4572;  // Positive = turret above floor. (18 inches) Tune on robot.
    // Turret angle offset (deg): 180 = turret encoder zero points opposite robot forward.
    public static final double kTurretOffsetYawDeg = 180.0;
  }

  /** Turret: convert robot-relative angle (deg) to motor position (rotations). */
  public static final class TurretKinematics {
    /** Encoder rotations when turret is at 0° (pointing forward). */
    public static final double kZeroPositionRotations = 0.0;
    /** Physical turret degrees per one motor rotation (positive = left). Tune on robot. */
    public static final double kDegreesPerMotorRotation = 8.5;
    /** Deadband (rotations): stop driving when within this of setpoint to reduce bouncing. */
    public static double kTurretDeadbandRotations = 0.04;
  }

  /** Turret position PID. Tune on robot. Lower P and max output reduce overshoot/bouncing. */
  public static final class TurretPID {
    public static final double kP = 0.05;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    /** Max percent output sent to motor to reduce overshoot. */
    public static final double kMaxOutput = 0.4;
  }

  /** Hood position PID. Tune on robot. */
  public static final class HoodPID {
    public static final double kP = 0.1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kMaxOutput = 0.35;
    public static final double kDeadband = 0.01;
  }

  /** Hood: convert angle (deg) to encoder position (rotations). */
  public static final class HoodKinematics {
    /** Encoder position when hood is at 0° (flat). */
    public static final double kZeroPosition = 0.0;
    /** Encoder rotations per degree of hood angle. Tune on robot. */
    public static final double kEncoderRotationsPerDegree = 0.2;
  }

  /** 2026 REBUILT: hub and shuttle targets in meters. WPILib field: +X toward red, +Y left when facing red. Update from AdvantageScope / game manual. */
  public static final class FieldTarget {
    /** Red alliance hub center (x, y, z). */
    public static final double HUB_RED_X_M = 11.966988;
    public static final double HUB_RED_Y_M = 4.0;
    public static final double HUB_RED_Z_M = 1.9;
    /** Blue alliance hub center. */
    public static final double HUB_BLUE_X_M = 4.574;
    public static final double HUB_BLUE_Y_M = 4.0;
    public static final double HUB_BLUE_Z_M = 1.9;
  }

  /**
   * Shuttle zones: near your hub = score (return 0), everywhere else = shuttle.
   * Shuttle target is on the same Y-half as the robot (low Y or high Y).
   */
  public static final class ShuttleZones {
    /** X boundary: red scores when X > this, blue scores when X < this. */
    public static final double RED_HUB_X_THRESHOLD = 11.0;
    public static final double BLUE_HUB_X_THRESHOLD = 4.676;
    /** Y divider: below = low-Y half, above = high-Y half. */
    public static final double Y_MIDLINE = 4.376;

    // --- Shuttle targets (low Y half): red's left / blue's right ---
    public static final double LOW_Y_RED_TARGET_X = 15.110;
    public static final double LOW_Y_RED_TARGET_Y = 2.017;
    public static final double LOW_Y_RED_TARGET_Z = 0;
    public static final double LOW_Y_BLUE_TARGET_X = 2;
    public static final double LOW_Y_BLUE_TARGET_Y = 2.017;
    public static final double LOW_Y_BLUE_TARGET_Z = 0;

    // --- Shuttle targets (high Y half): red's right / blue's left ---
    public static final double HIGH_Y_RED_TARGET_X = 15.110;
    public static final double HIGH_Y_RED_TARGET_Y = 5.963;
    public static final double HIGH_Y_RED_TARGET_Z = 0;
    public static final double HIGH_Y_BLUE_TARGET_X = 2;
    public static final double HIGH_Y_BLUE_TARGET_Y = 5.963;
    public static final double HIGH_Y_BLUE_TARGET_Z = 0;

    /**
     * Returns {zone, tx, ty, tz} if robot should shuttle, or {0} if near own hub (score).
     * zone = 1 (low Y half) or 2 (high Y half).
     */
    public static double[] getShuttleZone(double x, double y, boolean isRed) {
      // Near own hub → score, not shuttle
      if (isRed && x > RED_HUB_X_THRESHOLD) return new double[] { 0 };
      if (!isRed && x < BLUE_HUB_X_THRESHOLD) return new double[] { 0 };

      // Everywhere else → shuttle, pick target based on Y half
      if (y < Y_MIDLINE) {
        if (isRed) {
          return new double[] { 1, LOW_Y_RED_TARGET_X, LOW_Y_RED_TARGET_Y, LOW_Y_RED_TARGET_Z };
        } else {
          return new double[] { 1, LOW_Y_BLUE_TARGET_X, LOW_Y_BLUE_TARGET_Y, LOW_Y_BLUE_TARGET_Z };
        }
      } else {
        if (isRed) {
          return new double[] { 2, HIGH_Y_RED_TARGET_X, HIGH_Y_RED_TARGET_Y, HIGH_Y_RED_TARGET_Z };
        } else {
          return new double[] { 2, HIGH_Y_BLUE_TARGET_X, HIGH_Y_BLUE_TARGET_Y, HIGH_Y_BLUE_TARGET_Z };
        }
      }
    }
  }

  /** Limelight camera names (NetworkTables). Left and right chassis-mounted. When either sees an AprilTag, we update robot pose. */
  public static final class Limelights {
    public static final String[] NAMES = { "left", "right" };
  }

  /** Drive speed scale (0–1) applied only while shoot trigger is held. Lower = slower during feed. */
  public static final class ShootDriveLimits {
    public static final double kSpeedScale = 0.25;
    public static final double kRotScale = 0.5;
  }
  

  /** Spindex "ready" during Prep: low speed to hold ball in position without over-feeding. Tune on robot. */
  public static final class SpindexReady {
    /** Percent output (0–1) when prepping to shoot. */
    public static final double kPercent = 0.15;
  }

  public static class Locations {
    public static List<Location> locations = new ArrayList<Location>();

    // TODO: Set hub/shuttle (x,y,z) from official field drawings (meters). z = height of target center above field floor. */
    public Locations ()
    {
        addLocation("HUB_BLUE", FieldTarget.HUB_BLUE_X_M, FieldTarget.HUB_BLUE_Y_M, FieldTarget.HUB_BLUE_Z_M);
        addLocation("HUB_RED", FieldTarget.HUB_RED_X_M, FieldTarget.HUB_RED_Y_M, FieldTarget.HUB_RED_Z_M);
        // addLocation("SHUTTLE_BLUE", x, y, z);
        // addLocation("SHUTTLE_RED", x, y, z);
    }

    public static void addLocation(String name, double x, double y, double z)
    {
        Location loc = new Location();
        loc.Name = name;
        Pose3d BlueSpot = new Pose3d();
        var blueLation = new Translation3d(x,y,z);
        var blueTrans = new Transform3d(blueLation, Rotation3d.kZero);
        BlueSpot = BlueSpot.transformBy(blueTrans);
        loc.spot = BlueSpot;
        locations.add(loc);
        double[] DoubleArray = new double[3];
        DoubleArray[0] = blueLation.getX();
        DoubleArray[1]= blueLation.getY();
        DoubleArray[2]= blueLation.getZ();
        SmartDashboard.putNumberArray("Location - " + loc.Name, DoubleArray);
    }

    // Store a list of locations here we can access from Locations.<locationName>
    //Make a method to get locations by name.

    public static Location getLocationByName(String name) {
        if (name == null) return null;
        for (Location loc : locations) {
            if (name.equals(loc.Name)) return loc;
        }
        return null;
    }
}
}
