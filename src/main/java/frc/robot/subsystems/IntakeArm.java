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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Intake arm - two TalonFX x60 (IDs 14 left, 15 right). Extend/retract arms.
 * Motors spin in opposite directions; left = +percent, right = -percent for extend. Adjust if needed.
 */
public class IntakeArm extends SubsystemBase {
  private final TalonFX m_left = new TalonFX(Constants.Motors.INTAKE_ARM_LEFT);
  private final TalonFX m_right = new TalonFX(Constants.Motors.INTAKE_ARM_RIGHT);
  private final DutyCycleOut m_request = new DutyCycleOut(0).withEnableFOC(true);
  public int m_armState = 0;

  public IntakeArm() {
    var config = new com.ctre.phoenix6.configs.TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.withCurrentLimits(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(Amps.of(90))
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(Amps.of(37.5))
        .withSupplyCurrentLimitEnable(true));
    m_left.getConfigurator().apply(config);
    m_right.getConfigurator().apply(config);
  }

  /**
   * Set percent output directly to both motors.
   * Left motor = +percent, right motor = -percent so they spin opposite. Enforces limits.
   */
  public void setPercent(double percent) {
    if (!limitsEnabled()) {
      m_left.setControl(m_request.withOutput(percent));
      m_right.setControl(m_request.withOutput(-percent));
      return;
    }
    double pos = getPositionRotations();
    if (percent > 0 && pos >= Constants.IntakeArmLimits.kMaxPositionRotations) {
      stop();
      return;
    }
    if (percent < 0 && pos <= Constants.IntakeArmLimits.kMinPositionRotations) {
      stop();
      return;
    }
    m_left.setControl(m_request.withOutput(percent));
    m_right.setControl(m_request.withOutput(-percent));
  }

  private static boolean limitsEnabled() {
    return Constants.IntakeArmLimits.kMaxPositionRotations
        > Constants.IntakeArmLimits.kMinPositionRotations;
  }

  /** Position (rotations) from left motor. Use for limits and home. */
  public double getPositionRotations() {
    return m_left.getRotorPosition().getValueAsDouble();
  }

  /** Configured arm speed (always positive). Read from SmartDashboard or Constants default. */
  public double getSpeed() {
    return MathUtil.clamp(
        SmartDashboard.getNumber("IntakeArm/Speed", Constants.TestSpeeds.INTAKE_ARM),
        0.0, 1.0);
  }

  /** Extend arms out. Negative percent because wiring is swapped (extend = position decreases). */
  public void runExtend() {
    setPercent(-getSpeed());
  }

  /** Retract arms in. Positive percent (retract = position increases toward 0). */
  public void runRetract() {
    setPercent(getSpeed());
  }

  /** Run toward home position. Call from a command or periodic; stops when at home. */
  public void runTowardHome() {
    if (!limitsEnabled()) {
      stop();
      return;
    }
    double pos = getPositionRotations();
    double home = Constants.IntakeArmLimits.kHomePositionRotations;
    // TODO: Tune tolerance (rotations) for "at home" detection.
    double tol = 0.5;
    if (Math.abs(pos - home) <= tol) {
      stop();
      return;
    }
    double spd = getSpeed();
    setPercent(pos < home ? spd : -spd);
  }

  public boolean isAtHome() {
    if (!limitsEnabled()) return true;
    // TODO: Tune tolerance to match runTowardHome().
    double tol = 0.5;
    return Math.abs(getPositionRotations() - Constants.IntakeArmLimits.kHomePositionRotations) <= tol;
  }

  /** Stops motors but does not clear m_armState so driver bumper toggle (out/in) is preserved at limits. */
  public void stop() {
    m_left.setControl(m_request.withOutput(0));
    m_right.setControl(m_request.withOutput(0));
  }

  /** Stops motors and resets arm state to 0 (e.g. from stopAll). Call this when intentionally stopping. */
  public void stopAndResetState() {
    stop();
    m_armState = 0;
  }

  @Override
  public void periodic() {
    double pos = getPositionRotations();
    double motorOutput = m_left.get();
    if (limitsEnabled()) {
      if (pos <= Constants.IntakeArmLimits.kMinPositionRotations && motorOutput < 0) {
        stop();
      }
      if (pos >= Constants.IntakeArmLimits.kMaxPositionRotations && motorOutput > 0) {
        stop();
      }
    }
    SmartDashboard.putNumber("IntakeArm/Position", pos);
    SmartDashboard.putNumber("IntakeArm/LeftPercent", motorOutput);
    SmartDashboard.putNumber("IntakeArm/RightPercent", m_right.get());
    SmartDashboard.putNumber("IntakeArm/ArmState", m_armState);
    SmartDashboard.putBoolean("IntakeArm/LimitsEnabled", limitsEnabled());
    SmartDashboard.putBoolean("IntakeArm/AtHome", isAtHome());
  }
}
