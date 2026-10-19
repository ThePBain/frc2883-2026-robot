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

/** Elevator - TalonFX/Vortex ID 17. Manual run for testing. */
public class Elevator extends SubsystemBase {
  private final TalonFX m_motor = new TalonFX(Constants.Motors.ELEVATOR);
  private final DutyCycleOut m_request = new DutyCycleOut(0).withEnableFOC(true);

  public Elevator() {
    var config = new com.ctre.phoenix6.configs.TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.withCurrentLimits(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(Amps.of(65))
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(Amps.of(32.5))
        .withSupplyCurrentLimitEnable(true));
    m_motor.getConfigurator().apply(config);
  }

  public void setPercent(double percent) {
    m_motor.setControl(m_request.withOutput(percent));
  }

  public double getSpeed() {
    return MathUtil.clamp(
        SmartDashboard.getNumber("Elevator/Speed", Constants.TestSpeeds.ELEVATOR), -1.0, 1.0);
  }

  public void runForward() {
    setPercent(getSpeed());
  }

  public void runReverse() {
    setPercent(-getSpeed());
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
    SmartDashboard.putNumber("Elevator/Percent", m_motor.get());
    SmartDashboard.putNumber("Elevator/VelocityRPS", m_motor.getRotorVelocity().getValueAsDouble());
  }
}
