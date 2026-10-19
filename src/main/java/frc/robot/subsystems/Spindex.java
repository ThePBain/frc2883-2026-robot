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

/** Spindex (index) - TalonFX/Vortex ID 16. Manual run for testing. */
public class Spindex extends SubsystemBase {
  private final TalonFX m_motor = new TalonFX(Constants.Motors.SPINDEX);
  private final DutyCycleOut m_request = new DutyCycleOut(0).withEnableFOC(true);

  public Spindex() {
    var config = new com.ctre.phoenix6.configs.TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.withCurrentLimits(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(Amps.of(40))
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(Amps.of(20))
        .withSupplyCurrentLimitEnable(true));
    m_motor.getConfigurator().apply(config);
  }

  /** Run at percent output (positive = fwd). */
  public void setPercent(double percent) {
    m_motor.setControl(m_request.withOutput(percent));
  }

  /** Current speed (0–1) from SmartDashboard "Spindex/Speed". */
  public double getSpeed() {
    return MathUtil.clamp(
        SmartDashboard.getNumber("Spindex/Speed", Constants.TestSpeeds.SPINDEX), -1.0, 1.0);
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
    SmartDashboard.putNumber("Spindex/Percent", m_motor.get());
    SmartDashboard.putNumber("Spindex/VelocityRPS", m_motor.getRotorVelocity().getValueAsDouble());
  }
}
