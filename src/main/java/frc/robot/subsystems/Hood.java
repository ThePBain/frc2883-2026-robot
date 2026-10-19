// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Shooter Hood - Kraken X44 on TalonFX ID 19.
 * Position: 0 = lowest (flat), 1.5 = highest. Positive output = position increases (up).
 */
public class Hood extends SubsystemBase {
  private final TalonFX m_motor = new TalonFX(Constants.Motors.SHOOTER_HOOD);
  private final DutyCycleOut m_request = new DutyCycleOut(0).withEnableFOC(true);

  private final PIDController m_pid = new PIDController(
      Constants.HoodPID.kP, Constants.HoodPID.kI, Constants.HoodPID.kD);

  public Hood() {
    var config = new com.ctre.phoenix6.configs.TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.withCurrentLimits(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(Amps.of(60))
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(Amps.of(30))
        .withSupplyCurrentLimitEnable(true));
    m_motor.getConfigurator().apply(config);
    m_motor.setPosition(0);
    SmartDashboard.putNumber("Tuning/HoodKp", Constants.HoodPID.kP);
    SmartDashboard.putNumber("Tuning/HoodKi", Constants.HoodPID.kI);
    SmartDashboard.putNumber("Tuning/HoodKd", Constants.HoodPID.kD);
  }

  /** Set percent output. Positive = up (position increases), negative = down. Soft-limited. */
  public void setPercent(double percent) {
    if (!limitsEnabled()) {
      m_motor.setControl(m_request.withOutput(percent));
      return;
    }
    double pos = getPosition();
    if (pos <= Constants.HoodLimits.kMinPosition && percent < 0) {
      m_motor.setControl(m_request.withOutput(0));
      return;
    }
    if (pos >= Constants.HoodLimits.kMaxPosition && percent > 0) {
      m_motor.setControl(m_request.withOutput(0));
      return;
    }
    m_motor.setControl(m_request.withOutput(percent));
  }

  private static boolean limitsEnabled() {
    return Constants.HoodLimits.kMinPosition <= Constants.HoodLimits.kMaxPosition;
  }

  public double getSpeed() {
    return MathUtil.clamp(
        SmartDashboard.getNumber("Hood/Speed", Constants.TestSpeeds.HOOD), -1.0, 1.0);
  }

  /** Move hood up (positive output, position increases). */
  public void runUp() {
    setPercent(getSpeed());
  }

  /** Move hood down (negative output, position decreases). */
  public void runDown() {
    setPercent(-getSpeed());
  }

  public void stop() {
    m_motor.setControl(m_request.withOutput(0));
  }

  /** Current position (rotor rotations). For dashboard. */
  public double getPosition() {
    return m_motor.getRotorPosition().getValueAsDouble();
  }

  // --- Manual tuning target (operator D-pad) ---
  private static final double kNudgeStep = 0.05;
  private double m_manualTarget = Constants.HoodKinematics.kZeroPosition;

  /** Nudge manual target up (toward kMaxPosition = 1.5). */
  public void nudgeUp() {
    m_manualTarget = Math.min(Constants.HoodLimits.kMaxPosition, m_manualTarget + kNudgeStep);
  }

  /** Nudge manual target down (toward kMinPosition = 0). */
  public void nudgeDown() {
    m_manualTarget = Math.max(Constants.HoodLimits.kMinPosition, m_manualTarget - kNudgeStep);
  }

  /** Current manual target (rotations). TurretAim uses this when IDLE. */
  public double getManualTarget() {
    return m_manualTarget;
  }

  // --- Setpoint control (PID) ---
  private Double m_positionSetpoint = null;

  /** Set position setpoint (rotations). When set, periodic() drives hood to this position. */
  public void setPositionSetpoint(double position) {
    m_positionSetpoint = position;
  }

  /** Clear setpoint so manual percent control is used again. */
  public void clearPositionSetpoint() {
    m_positionSetpoint = null;
  }

  public boolean hasPositionSetpoint() {
    return m_positionSetpoint != null;
  }

  @Override
  public void periodic() {
    double pos = getPosition();
    if (m_positionSetpoint != null) {
      m_pid.setP(SmartDashboard.getNumber("Tuning/HoodKp", Constants.HoodPID.kP));
      m_pid.setI(SmartDashboard.getNumber("Tuning/HoodKi", Constants.HoodPID.kI));
      m_pid.setD(SmartDashboard.getNumber("Tuning/HoodKd", Constants.HoodPID.kD));
      m_pid.setSetpoint(m_positionSetpoint);
      double out = m_pid.calculate(pos);
      if (Math.abs(m_positionSetpoint - pos) < Constants.HoodPID.kDeadband) {
        out = 0;
        m_pid.reset();
      }
      out = MathUtil.clamp(out, -Constants.HoodPID.kMaxOutput, Constants.HoodPID.kMaxOutput);
      if (limitsEnabled()) {
        if (pos <= Constants.HoodLimits.kMinPosition && out < 0) out = 0;
        if (pos >= Constants.HoodLimits.kMaxPosition && out > 0) out = 0;
      }
      setPercent(out);
    }
    SmartDashboard.putNumber("Hood/Percent", m_motor.get());
    SmartDashboard.putNumber("Hood/Position", pos);
    SmartDashboard.putNumber("Hood/ManualTarget", m_manualTarget);
    SmartDashboard.putNumber("Hood/Error", m_positionSetpoint != null ? m_positionSetpoint - pos : 0);
    SmartDashboard.putBoolean("Hood/LimitsEnabled", limitsEnabled());
    SmartDashboard.putBoolean("Hood/SetpointActive", m_positionSetpoint != null);
  }
}
