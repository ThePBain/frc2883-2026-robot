// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Shooter Flywheel - Left Neo Vortex ID 20, Right Neo Vortex ID 21 (Spark Flex).
 *  Controlled only by TurretAim (shot calculator table). Velocity PID keeps speed constant under load. */
public class Flywheel extends SubsystemBase {
  private final SparkFlex m_left = new SparkFlex(Constants.Motors.FLYWHEEL_LEFT, MotorType.kBrushless);
  private final SparkFlex m_right = new SparkFlex(Constants.Motors.FLYWHEEL_RIGHT, MotorType.kBrushless);
  private final SparkClosedLoopController m_leftController = m_left.getClosedLoopController();
  private final SparkClosedLoopController m_rightController = m_right.getClosedLoopController();

  /** Speed 0–1, maps to RPM range for velocity setpoint. */
  private double m_speed = Constants.TestSpeeds.FLYWHEEL;
  /** When non-null, shot calculator is driving; use this RPM instead of m_speed. */
  private Double targetRPM = null;

  public Flywheel() {
    var closedLoop = new ClosedLoopConfig()
        .pid(
            Constants.FlywheelVelocity.kP,
            Constants.FlywheelVelocity.kI,
            Constants.FlywheelVelocity.kD).velocityFF(Constants.FlywheelVelocity.kFF)
        .outputRange(-1.0, 1.0);
    var config = new SparkFlexConfig()
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(68)
        .apply(closedLoop);

    m_left.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    m_right.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /** Target RPM: vision override when aiming, else m_speed (0–1) maps to MIN_RPM..MAX_RPM. */
  public double getTargetRpm() {
    return targetRPM;
  }

  /** Set RPM setpoint from shot calculator. Apply immediately if running. */
  public void setTargetRpm(double rpm) {
    targetRPM = rpm;
    setVelocityRpm(rpm);
  }

  /** Run both flywheels at velocity setpoint (RPM). PID increases power under load. */
  private void setVelocityRpm(double rpm) {
    targetRPM = rpm;
    m_leftController.setSetpoint(-rpm, ControlType.kVelocity);
    m_rightController.setSetpoint(rpm, ControlType.kVelocity);
  }

  public void stop() {
    m_left.set(0);
    m_right.set(0);
  }

  /** Current speed (0–1). */
  public double getSpeed() {
    return m_speed;
  }

  @Override
  public void periodic() {
    double rpm = targetRPM;
    setVelocityRpm(rpm);
    double leftRpm = m_left.getEncoder().getVelocity();
    double rightRpm = m_right.getEncoder().getVelocity();
    SmartDashboard.putNumber("Flywheel/Speed", m_speed);
    SmartDashboard.putNumber("Flywheel/TargetRPM", getTargetRpm());
    SmartDashboard.putNumber("Flywheel/LeftRPM", leftRpm);
    SmartDashboard.putNumber("Flywheel/RightRPM", rightRpm);
    SmartDashboard.putNumber("Flywheel/Percent", m_left.get());
  }
}
