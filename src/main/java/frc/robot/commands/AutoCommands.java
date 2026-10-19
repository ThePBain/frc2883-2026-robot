package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.RobotState.VisionCollectionMode;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeWheels;
import frc.robot.subsystems.Spindex;

/**
 * Factory for PathPlanner event marker commands.
 * Register these in RobotContainer via NamedCommands.registerCommand().
 */
public final class AutoCommands {

  private AutoCommands() {}

  /**
   * Auto Aim: sets the autoAimActive flag so TurretAim (the default command)
   * aims turret, hood, and flywheel at the target. No subsystem requirements
   * needed -- TurretAim is already running and reads this flag.
   */
  public static Command autoAim() {
    return Commands.startEnd(
        () -> RobotState.s_autoAimActive = true,
        () -> RobotState.s_autoAimActive = false
    );
  }

  /**
   * Auto Shoot: feeds the ball by running spindex forward and elevator reverse.
   * Runs until interrupted (use withTimeout or as part of a sequence).
   */
  public static Command autoShoot(Spindex spindex, Elevator elevator) {
    return Commands.run(() -> {
      spindex.runForward();
      elevator.runReverse();
    }, spindex, elevator).finallyDo(interrupted -> {
      spindex.stop();
      elevator.stop();
    });
  }

  /**
   * Intake Out: sets flags so default commands extend arm and run wheels in.
   * Finishes immediately (runOnce) so path continues without stopping.
   */
  public static Command intakeOut(IntakeArm intakeArm, IntakeWheels intakeWheels) {
    return Commands.runOnce(() -> {
      intakeArm.m_armState = 1;
      intakeWheels.m_intakeWheelsOn = true;
    });
  }

  /**
   * Intake In: sets flags so default commands retract arm and stop wheels.
   * Finishes immediately (runOnce) so path continues without stopping.
   */
  public static Command intakeIn(IntakeArm intakeArm, IntakeWheels intakeWheels) {
    return Commands.runOnce(() -> {
      intakeArm.m_armState = 2;
      intakeWheels.m_intakeWheelsOn = false;
    });
  }

  /**
   * Collect Pose: briefly enables Limelight vision pose updates for odometry correction.s
   * Turns on collection, waits a short period for the Limelights to push measurements,
   * then turns collection back off and finishes.
   */
  public static Command collectPose() {
    return Commands.sequence(
        Commands.runOnce(() -> RobotState.s_ShootAuth = VisionCollectionMode.COLLECT),
        Commands.waitSeconds(.75),
        Commands.runOnce(() -> RobotState.s_ShootAuth = VisionCollectionMode.IGNORE)
    );
  }
}
