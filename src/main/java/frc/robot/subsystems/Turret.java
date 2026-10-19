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

/** Turret - TalonFX/Vortex ID 18. Manual percent for testing. */
public class Turret extends SubsystemBase {
  private final TalonFX m_motor = new TalonFX(Constants.Motors.TURRET);
  private final DutyCycleOut m_request = new DutyCycleOut(0).withEnableFOC(true);

  public Turret() {
    var config = new com.ctre.phoenix6.configs.TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.withCurrentLimits(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(Amps.of(45))
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(Amps.of(22.5))
        .withSupplyCurrentLimitEnable(true));
    m_motor.getConfigurator().apply(config);
    m_motor.setPosition(0);  // Zero encoder at startup; current physical position = logical 0
    SmartDashboard.putNumber("Tuning/TurretKp", Constants.TurretPID.kP);
    SmartDashboard.putNumber("Tuning/TurretKi", Constants.TurretPID.kI);
    SmartDashboard.putNumber("Tuning/TurretKd", Constants.TurretPID.kD);
  }

  /** Percent: positive = left, negative = right. Limited so turret does not pass min/max position. Clears position setpoint. */
  public void setPercent(double percent) {
    m_positionSetpoint = null;
    if (!limitsEnabled()) {
      m_motor.setControl(m_request.withOutput(0));
      return;
    }
    // TODO: Limits from Constants.TurretLimits; set kMinPositionRotations/kMaxPositionRotations on robot.
    double pos = m_motor.getRotorPosition().getValueAsDouble();
    if (pos <= Constants.TurretLimits.kMinPositionRotations && percent < 0) {
      m_motor.setControl(m_request.withOutput(0));
      return;
    }
    if (pos >= Constants.TurretLimits.kMaxPositionRotations && percent > 0) {
      m_motor.setControl(m_request.withOutput(0));
      return;
    }
    m_motor.setControl(m_request.withOutput(percent*Constants.TestSpeeds.TURRET));
  }

  private static boolean limitsEnabled() {
    return Constants.TurretLimits.kMaxPositionRotations > Constants.TurretLimits.kMinPositionRotations;
  }

  public double getSpeed() {
    return MathUtil.clamp(
        SmartDashboard.getNumber("Turret/Speed", Constants.TestSpeeds.TURRET), -1.0, 1.0);
  }

  public void runLeft() {
    setPercent(getSpeed());
  }

  public void runRight() {
    setPercent(-getSpeed());
  }

  public void stop() {
    m_motor.setControl(m_request.withOutput(0));
  }

  /** Current position (rotor rotations). For dashboard. */
  public double getPosition() {
    return m_motor.getRotorPosition().getValueAsDouble();
  }

  // --- PID setpoint control for shot calculator (vision aim) ---
  private final PIDController m_pid = new PIDController(
      Constants.TurretPID.kP,
      Constants.TurretPID.kI,
      Constants.TurretPID.kD);
  private Double m_positionSetpoint = null;

  /** Set position setpoint (rotations). When set, periodic() drives turret to this position. */
  public void setPositionSetpoint(double rotations) {
    m_positionSetpoint = rotations;
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
      // Read tunable gains from SmartDashboard (or use Constants defaults)
      m_pid.setP(SmartDashboard.getNumber("Tuning/TurretKp", Constants.TurretPID.kP));
      m_pid.setI(SmartDashboard.getNumber("Tuning/TurretKi", Constants.TurretPID.kI));
      m_pid.setD(SmartDashboard.getNumber("Tuning/TurretKd", Constants.TurretPID.kD));
      m_pid.setSetpoint(m_positionSetpoint);
      double out = m_pid.calculate(pos);
      double deadband = Constants.TurretKinematics.kTurretDeadbandRotations;
      if (Math.abs(m_positionSetpoint - pos) < deadband) {
        out = 0.0;
        m_pid.reset();
      }
      out = MathUtil.clamp(out, -Constants.TurretPID.kMaxOutput, Constants.TurretPID.kMaxOutput);
      // Apply limits: don't drive past soft limits
      if (limitsEnabled()) {
        if (pos <= Constants.TurretLimits.kMinPositionRotations && out < 0) out = 0;
        if (pos >= Constants.TurretLimits.kMaxPositionRotations && out > 0) out = 0;
      }
      setPercent(out);
    }
    SmartDashboard.putNumber("Turret/Percent", m_motor.get());
    SmartDashboard.putNumber("Turret/Position", pos);
    SmartDashboard.putNumber("Turret/Error", m_positionSetpoint != null ? m_positionSetpoint - pos : 0);
    SmartDashboard.putBoolean("Turret/LimitsEnabled", limitsEnabled());
    SmartDashboard.putBoolean("Turret/SetpointActive", m_positionSetpoint != null);
  }
}
