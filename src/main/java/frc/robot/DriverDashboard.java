// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;

/**
 * Builds and updates a single "Driver" tab in Shuffleboard (and any dashboard reading
 * NetworkTables) so all key robot data appears in one place without manual widget setup.
 *
 * @see <a href="https://docs.wpilib.org/en/stable/docs/software/dashboards/shuffleboard/getting-started/shuffleboard-displaying-data.html">Shuffleboard - Displaying data</a>
 * @see <a href="https://docs.wpilib.org/en/stable/docs/software/dashboards/shuffleboard/layouts-with-code/sending-data.html">Shuffleboard - Sending data</a>
 * @see <a href="https://docs.wpilib.org/en/stable/docs/software/dashboards/dashboard-intro.html">Choosing a Dashboard</a>
 */
public class DriverDashboard {
  private final ShuffleboardTab tab = Shuffleboard.getTab("Driver");

  // Swerve
  private final GenericEntry poseX = tab.add("Pose X (m)", 0).withWidget(BuiltInWidgets.kNumberBar).getEntry();
  private final GenericEntry poseY = tab.add("Pose Y (m)", 0).withWidget(BuiltInWidgets.kNumberBar).getEntry();
  private final GenericEntry headingDeg = tab.add("Heading (deg)", 0).withWidget(BuiltInWidgets.kDial).getEntry();

  // Intake (driver-only subsystems for ball pickup)
  private final GenericEntry intakeWheelsPercent = tab.add("Intake Wheels %", 0).getEntry();
  private final GenericEntry intakeArmPosition = tab.add("Intake Arm Pos", 0).getEntry();
  private final GenericEntry intakeArmAtHome = tab.add("Intake At Home", false).getEntry();

  public DriverDashboard() {}

  /**
   * Update all Driver tab entries from the robot. Call this from {@link edu.wpi.first.wpilibj.TimedRobot#robotPeriodic()}.
   */
  public void update(RobotContainer rc) {
    // Swerve pose (from CTRE drivetrain state)
    try {
      var state = rc.drivetrain.getState();
      Pose2d pose = state.Pose;
      poseX.setDouble(pose.getX());
      poseY.setDouble(pose.getY());
      headingDeg.setDouble(pose.getRotation().getDegrees());
    } catch (Exception ignored) {
      // Drivetrain may not be ready
    }

    // Intake
    intakeWheelsPercent.setDouble(rc.intakeWheels.getOutput());
    intakeArmPosition.setDouble(rc.intakeArm.getPositionRotations());
    intakeArmAtHome.setBoolean(rc.intakeArm.isAtHome());
  }
}
