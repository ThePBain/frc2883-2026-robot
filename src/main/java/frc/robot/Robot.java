// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degree;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotState.VisionCollectionMode;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        DataLogManager.start();
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        // Only run Hoot replay when actually replaying; on real hardware this can interfere with live CAN/joystick.
        if (edu.wpi.first.wpilibj.RobotBase.isSimulation()) {
            m_timeAndJoystickReplay.update();
        }
        CommandScheduler.getInstance().run();
        PushSmartDashValues();
    }

    public void PushSmartDashValues()
    {
        var drivetrain = m_robotContainer.drivetrain;
        double pigeonYaw = drivetrain.getPigeon2().getYaw().getValue().in(Degree);
        double poseYaw = drivetrain.getState().Pose.getRotation().getDegrees();
        double rawHeading = drivetrain.getState().RawHeading.getDegrees();
        SmartDashboard.putNumber("YawDebug/PigeonYaw", pigeonYaw);
        SmartDashboard.putNumber("YawDebug/PoseRotation", poseYaw);
        SmartDashboard.putNumber("YawDebug/RawHeading", rawHeading);
        SmartDashboard.putNumber("YawDebug/PoseVsPigeonDelta", poseYaw - pigeonYaw);
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_robotContainer.updateHubTargetForAlliance();
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        frc.robot.RobotState.s_ShootAuth = VisionCollectionMode.IGNORE;
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {
        frc.robot.RobotState.s_autoAimActive = false;
    }

    @Override
    public void teleopInit() {
        m_robotContainer.updateHubTargetForAlliance();
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
        RobotContainer.s_teleopStartTimeSeconds = Timer.getFPGATimestamp();
        frc.robot.RobotState.s_ShootAuth = VisionCollectionMode.COLLECT;
    }

    @Override
    public void teleopPeriodic() {
        // Force Idle to drivetrain for first kForceIdleSeconds so we can tell if spin is from commands or hardware/config.
        m_robotContainer.forceDriveIdleForSettling();
    }

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        frc.robot.RobotState.s_ShootAuth = VisionCollectionMode.COLLECT;
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
