package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.Constants.Locations;
import frc.robot.RobotState.ShootMode;
import frc.robot.constants.Location;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Turret;
import frc.robot.vision.ShotCalculator;

/**
 * Runs constantly as turret default. Right stick = aim at hub, Left stick = aim at shuttle.
 * Hood state: when RobotState is PREPARING or SHOOTING, hood at aim setpoint; when IDLE, hood at zero.
 * Spindex/loader are handled by a separate trigger subsystem.
 */
public class TurretAim extends Command {
  private final CommandSwerveDrivetrain m_drivetrain;
  private final Turret m_turret;
  private final Hood m_hood;
  private final Flywheel m_flywheel;
  private final ShotCalculator m_shotCalculator;
  private final CommandXboxController m_operatorController;
  private final CommandXboxController m_driverController;

  public TurretAim(
      CommandSwerveDrivetrain drivetrain,
      Turret turret,
      Hood hood,
      Flywheel flywheel,
      ShotCalculator shotCalculator,
      CommandXboxController operatorController,
      CommandXboxController driverController) {
    m_drivetrain = drivetrain;
    m_turret = turret;
    m_hood = hood;
    m_flywheel = flywheel;
    m_shotCalculator = shotCalculator;
    m_operatorController = operatorController;
    m_driverController = driverController;
    addRequirements(turret, hood, flywheel);
  }

  @Override
  public void initialize() {
    // Don't clear override - no manual controls; shot calculator is sole source
  }

  @Override
  public void execute() {

    boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    
    XboxController hid = (XboxController) m_operatorController.getHID();
    boolean score = false;
    boolean shuttle = false;
    score |= hid.getLeftTriggerAxis() > 0.1;  // Operator left trigger = aim (same as driver LT)

    XboxController driverHid = (XboxController) m_driverController.getHID();
    score |= driverHid.getLeftTriggerAxis() > 0.1;  // Driver left trigger = aim
    score |= RobotState.s_autoAimActive;  // Auto command can trigger aiming

    Pose2d robotPose = m_drivetrain.getState().Pose;
    double[] shuttleZone = Constants.ShuttleZones.getShuttleZone(
        robotPose.getX(), robotPose.getY(), isRed);

    // Pick target: use shuttle zone target when in a zone, otherwise use hub
    boolean inShuttleZone = shuttleZone[0] > 0;
    String hubName = isRed ? "HUB_RED" : "HUB_BLUE";
    double tx, ty, tz;
    if (inShuttleZone) {
        tx = shuttleZone[1];
        ty = shuttleZone[2];
        tz = shuttleZone[3];
        shuttle = score;
    } else {
      Location loc = Locations.getLocationByName(hubName);
      tx = loc.spot.getX();
      ty = loc.spot.getY();
      tz = loc.spot.getZ();
    }
    var sp = m_shotCalculator.getSetpointsFromPose(m_drivetrain, m_turret, tx, ty, tz, inShuttleZone);
    SmartDashboard.putBoolean("Aim/AlliancePresent", DriverStation.getAlliance().isPresent());
    SmartDashboard.putBoolean("Aim/IsRed", isRed);
    SmartDashboard.putString("Aim/HubName", inShuttleZone ? "shuttle" : hubName);
    SmartDashboard.putNumber("Aim/ShuttleZone", shuttleZone[0]);
    SmartDashboard.putString("Aim/Mode", inShuttleZone ? "Shuttle" : "Score");
    SmartDashboard.putBoolean("Aim/Valid", sp.valid);
    SmartDashboard.putNumber("Aim/DistanceM", sp.distanceMeters);
    SmartDashboard.putNumber("Aim/TurretSetpoint", sp.turretRotations);
    SmartDashboard.putNumber("Aim/HoodSetpoint", sp.hoodPosition);
    SmartDashboard.putNumber("Aim/FlywheelRpm", sp.flywheelRpm);

    if (!score && !shuttle) {
      if (RobotState.getShootMode() != ShootMode.SHOOTING) {
        RobotState.setShootMode(ShootMode.IDLE);
        m_hood.setPositionSetpoint(m_hood.getManualTarget());
        m_turret.clearPositionSetpoint();
        m_turret.stop();
        m_flywheel.setTargetRpm(Constants.FLYWHEEL_SPEED_MIN);
      }
      // else: SHOOTING, keep turret/hood/flywheel setpoints (trigger subsystem runs spindex/loader)
      SmartDashboard.putString("Aim/Mode", "—");
      //SmartDashboard.putBoolean("Aim/Valid", false);
      return;
    }

    // Only set PREPARING if shoot trigger isn't held (SHOOTING takes precedence for drive limit)
    if (RobotState.getShootMode() != ShootMode.SHOOTING) {
      RobotState.setShootMode(ShootMode.PREPARING);
    }

    if (sp.valid) {
      m_turret.setPositionSetpoint(sp.turretRotations);
      m_hood.setPositionSetpoint(sp.hoodPosition);
      m_flywheel.setTargetRpm(sp.flywheelRpm);
    } else {
      m_hood.setPositionSetpoint(m_hood.getManualTarget());
      m_turret.clearPositionSetpoint();
      m_turret.stop();
      m_flywheel.setTargetRpm(Constants.FLYWHEEL_SPEED_MIN);
      //m_flywheel.stopIfNotManual();
    }
  }

  @Override
  public void end(boolean interrupted) {
    if(interrupted) {
      // If interrupted, don't mess with shoot mode or setpoints - just stop turret and flywheel motion
      m_turret.stop();
      m_flywheel.setTargetRpm(Constants.FLYWHEEL_SPEED_MIN);
      SmartDashboard.putString("AIM INTERRUPT", "TRUE");
      return;
    }
    RobotState.setShootMode(ShootMode.IDLE);
    m_hood.setPositionSetpoint(Constants.HoodKinematics.kZeroPosition);
    m_turret.clearPositionSetpoint();
    m_turret.stop();
    m_flywheel.setTargetRpm(Constants.FLYWHEEL_SPEED_MIN);
    SmartDashboard.putString("Aim/Mode", "—");
    SmartDashboard.putBoolean("Aim/Valid", false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
