package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.RobotContainer;
import static frc.robot.RobotState.VisionCollectionMode;


/**
 * Limelight: corrects swerve odometry when it sees AprilTags. We do NOT need to be looking at
 * the hub (or shuttle) to shoot—any tag improves our pose. Shooter uses pose + fixed target
 * locations (Locations) to aim. This just keeps the pose accurate.
 */
public class Limelight extends SubsystemBase {

    public String LL_Name = "limelight";

    public boolean IsRed = true;

    /** When true and getTV(), we push vision pose to drivetrain to correct odometry. */
    public boolean ShouldLimelightCollectPose = true;

    public Pigeon2 m_gyro;

    public CommandSwerveDrivetrain m_drivetrainRef;

    public double getTX() {
        return LimelightHelpers.getTX(LL_Name);
    }
    public double getTY() {
        return LimelightHelpers.getTY(LL_Name);
    }
    public double getTA() {
        return LimelightHelpers.getTA(LL_Name);
    }
    public boolean getTV()
    {
      return LimelightHelpers.getTV(LL_Name);
    }

    public Pose3d getRobotRelativeToTarget()
    {
      return LimelightHelpers.getBotPose3d_TargetSpace(LL_Name);
    }

    public Pose3d getBotPose3d_TargetSpace() {
        return LimelightHelpers.getBotPose3d_TargetSpace(LL_Name);
    }


    /** Use wpiBlue (standard field frame) for pose so it matches HUB_RED/HUB_BLUE coords. isRed ignored for pose. */
    public PoseEstimate getBotPose3d(boolean isRed, double curYaw) {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue(LL_Name);
    }

    @Override
    public void periodic() {
        if(frc.robot.RobotState.s_ShootAuth == VisionCollectionMode.COLLECT && ShouldLimelightCollectPose && getTV()) {
        GetVisonMeasurementAndTimestamp(IsRed);
      }
    }

    /** Push vision pose to drivetrain so odometry stays accurate. Any AprilTag (not just hub) helps. */
    public void GetVisonMeasurementAndTimestamp(boolean isRed) {
        // SetRobotOrientation expects DEGREES (LimelightHelpers Javadoc).
        double robotYawDeg = m_gyro.getYaw().getValue().in(Degree);
        PoseEstimate limelightMeasurement = getBotPose3d(isRed, robotYawDeg);
        // Skip invalid pose (e.g. empty botpose array) so we don't pull odometry to origin or garbage.
        if (!Boolean.TRUE.equals(LimelightHelpers.validPoseEstimate(limelightMeasurement))) {
            return;
        }

        // --- Smart per-camera std devs so both Limelights can contribute together ---
        // Base trust values (meters / radians) for a "good" multi-tag, close shot.
        double baseXY = 0.3;
        double baseTheta = Math.toRadians(30.0);

        int tags = limelightMeasurement.tagCount;
        double avgDist = limelightMeasurement.avgTagDist;

        // More tags → more trust (smaller std dev). Clamp between 1 and ~3.
        double tagFactor = tags > 0 ? 1.0 / Math.min(tags, 3) : 3.0;
        // Further tags → less trust. At 2m, factor ≈ 1; increases slowly with distance.
        double distFactor = 1.0 + Math.max(0.0, avgDist - 2.0) * 0.25;
        // Slightly down‑weight the right camera so if both see the same tags,
        // the left cam is primary and right is a helper.
        double camFactor = LL_Name.toLowerCase().contains("right") ? 1.3 : 1.0;

        double stdXY = baseXY * tagFactor * distFactor * camFactor;
        double stdTheta = baseTheta * tagFactor * distFactor * camFactor;

        // Std devs [x_m, y_m, theta_rad]; Phoenix expects radians for theta.
        m_drivetrainRef.setVisionMeasurementStdDevs(
            VecBuilder.fill(stdXY, stdXY, stdTheta));

        double visionTimestampSeconds = Utils.getCurrentTimeSeconds() - (limelightMeasurement.latency / 1000.0);
        m_drivetrainRef.addVisionMeasurement(limelightMeasurement.pose, visionTimestampSeconds);
    }

    public Limelight(String name, RobotContainer RobotContainer, boolean isRed, boolean shouldCollectPose) {
        this.LL_Name = name;
        this.m_drivetrainRef = RobotContainer.drivetrain;
        this.IsRed = isRed;
        this.ShouldLimelightCollectPose = shouldCollectPose;
        this.m_gyro = RobotContainer.drivetrain.getPigeon2();
        LimelightHelpers.SetThrottle(name, 2);

    }

}