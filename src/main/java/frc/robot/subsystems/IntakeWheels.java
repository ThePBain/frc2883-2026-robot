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

/** Intake wheels (roller) - TalonFX x44 ID 23. Suck in / spit out. */
public class IntakeWheels extends SubsystemBase {
  private final TalonFX m_motor = new TalonFX(Constants.Motors.INTAKE_WHEELS);
  private final DutyCycleOut m_request = new DutyCycleOut(0).withEnableFOC(true);
  public boolean m_intakeWheelsOn = false;

  public IntakeWheels() {
    var config = new com.ctre.phoenix6.configs.TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.withCurrentLimits(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(Amps.of(52.5))
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(Amps.of(30))
        .withSupplyCurrentLimitEnable(true));
    m_motor.getConfigurator().apply(config);
  }

  /** Run at percent output. Positive = in (suck), negative = out (spit). Adjust sign if needed. */
  public void setPercent(double percent) {
    m_motor.setControl(m_request.withOutput(percent));
  }

  public double getSpeed() {
    return MathUtil.clamp(
        SmartDashboard.getNumber("IntakeWheels/Speed", Constants.TestSpeeds.INTAKE_WHEELS),
        -1.0, 1.0);
  }

  /** Run wheels to suck notes in. (Direction reversed to match physical wiring.) */
  public void runIn() {
    setPercent(-getSpeed());
  }

  /** Run wheels to spit out. */
  public void runOut() {
    setPercent(getSpeed());
  }

  public void stop() {
    m_motor.setControl(m_request.withOutput(0));
  }

  /** Current motor output (-1 to 1). For dashboard. */
  public double getOutput() {
    return m_motor.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("IntakeWheels/Percent", m_motor.get());
    SmartDashboard.putNumber("IntakeWheels/VelocityRPS", m_motor.getRotorVelocity().getValueAsDouble());
  }
}
