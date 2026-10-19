package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.RobotState.ShootMode;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Turret;

/**
 * Default turret command when not aiming. Keeps hood at zero, turret and flywheel stopped.
 */
public class TurretIdle extends Command {
  private final Turret m_turret;
  private final Hood m_hood;
  private final Flywheel m_flywheel;

  public TurretIdle(Turret turret, Hood hood, Flywheel flywheel) {
    m_turret = turret;
    m_hood = hood;
    m_flywheel = flywheel;
    addRequirements(turret, hood, flywheel);
  }

  @Override
  public void execute() {
    if (RobotState.getShootMode() != ShootMode.SHOOTING) {
      RobotState.setShootMode(ShootMode.IDLE);
    }
    m_hood.setPositionSetpoint(Constants.HoodKinematics.kZeroPosition);
    m_turret.clearPositionSetpoint();
    m_turret.stop();
    m_flywheel.setTargetRpm(Constants.FLYWHEEL_SPEED_MIN);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
