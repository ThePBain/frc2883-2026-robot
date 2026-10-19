package frc.robot.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.RobotState.ShootMode;
import frc.robot.RobotState.VisionCollectionMode;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Turret;

public class CollectMode extends Command {
  private VisionCollectionMode Collect;

  public CollectMode(VisionCollectionMode collect) {
    this.Collect = collect;
  }

  @Override
  public void execute() {
    RobotState.s_ShootAuth = this.Collect;
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
