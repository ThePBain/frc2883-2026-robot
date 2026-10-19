// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.TurretAim;
import frc.robot.Constants.Locations;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;

import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeWheels;
import frc.robot.subsystems.Spindex;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Turret;
import frc.robot.vision.ShotCalculator;

public class RobotContainer {
    //DRIVE CONSTANTS
    public final static double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public final static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    /** Larger deadband (0.08) to avoid erratic spin from stick/gyro noise; reduce if drive feels sluggish. */
    public final static double Deadband = 0.08;
    public final static double RotDeadband = 0.08;

    /** Teleop start time (seconds) so we can force zero for a settling period after enable. */
    public static volatile double s_teleopStartTimeSeconds = Double.NEGATIVE_INFINITY;

    /** Call at match start (auto/teleop init) to refresh hub target from current alliance. */
    public void updateHubTargetForAlliance() {
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            RobotState.setSelectedTargetName("HUB_RED");
        } else {
            RobotState.setSelectedTargetName("HUB_BLUE");
        }
    }
    /** How long (seconds) to hold drive at zero after enable before accepting joystick. Tune if it feels too long/short. */
    public static final double kDriveSettlingSeconds = 0.5;
    /** If > 0, teleopPeriodic will force Idle to drivetrain for this many seconds after enable (overrides commands). Use to test: if it still spins, bug is in swerve config/hardware. Set to 0 to disable. */
    public static final double kForceIdleSeconds = 2.0;

    /* Field-centric using RawHeading; left bumper sets offset so current heading = joystick forward. */
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private static final SwerveRequest.Idle s_idle = new SwerveRequest.Idle();

    // Previous-cycle drive outputs for acceleration limiting
    private double m_prevVx = 0, m_prevVy = 0, m_prevRot = 0;

    

    public final IntakeWheels intakeWheels = new IntakeWheels();
    public final IntakeArm intakeArm = new IntakeArm();
    public final Spindex spindex = new Spindex();
    public final Elevator elevator = new Elevator();
    public final Turret turret = new Turret();
    public final Hood hood = new Hood();
    public final Flywheel flywheel = new Flywheel();

    private final ShotCalculator m_shotCalculator = new ShotCalculator();



    //Logging
    private final Telemetry logger = new Telemetry(MaxSpeed);

    // TODO: Verify controller ports (0 = driver, 1 = operator) match your Driver Station config.
    private final CommandXboxController m_controller = new CommandXboxController(0);
    private final CommandXboxController m_operatorController = new CommandXboxController(1);

    // SUBSYSTEMS — Motor IDs and tunables are in Constants.java only
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Limelight LL_Left;
    public final Limelight LL_Right;
    /** Single shooter: flywheel, hood, turret with velocity compensation (shoot on the move). */

    //DRIVER STATION UI STUFF
    public final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        SmartDashboard.putNumber("Robot/MaxSpeed", MaxSpeed);
        SmartDashboard.putNumber("Robot/MaxAngularRate", MaxAngularRate);
        // ADJUST: Selected target can be set here from alliance, or later from operator (e.g. D-pad)
        // Both Limelights can now contribute to pose; std devs are scaled per-camera/tag quality in Limelight.java.
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            LL_Left = new Limelight("limelight-left", this, true, true);   // left pushes pose
            LL_Right = new Limelight("limelight-right", this, true, true); // right also pushes pose (with its own std devs)
            RobotState.setSelectedTargetName("HUB_RED");
        } else {
            LL_Left = new Limelight("limelight-left", this, false, true);
            LL_Right = new Limelight("limelight-right", this, false, true);
            RobotState.setSelectedTargetName("HUB_BLUE");
        }


        configureBindings();

        // Register PathPlanner named commands (event markers) BEFORE building auto chooser.
        // AutoAim/CollectPose have no subsystem requirements -- they just set flags
        // that TurretAim (the default command) reads, avoiding scheduling conflicts.
        NamedCommands.registerCommand("AutoAim", AutoCommands.autoAim());
        NamedCommands.registerCommand("AutoShoot",
            AutoCommands.autoShoot(spindex, elevator));
        NamedCommands.registerCommand("IntakeOut",
            AutoCommands.intakeOut(intakeArm, intakeWheels));
        NamedCommands.registerCommand("IntakeIn",
            AutoCommands.intakeIn(intakeArm, intakeWheels));
        // IntakeStop: ensure intake is off when auto ends
        NamedCommands.registerCommand("IntakeStop", Commands.runOnce(() -> {
            intakeArm.m_armState = 0;
            intakeWheels.m_intakeWheelsOn = false;
        }));
        NamedCommands.registerCommand("CollectPose",
            AutoCommands.collectPose());

        // EventTrigger bindings for event markers ON PATHS (separate from NamedCommands)
        new EventTrigger("AutoAim").onTrue(AutoCommands.autoAim());
        new EventTrigger("AutoShoot").onTrue(AutoCommands.autoShoot(spindex, elevator));
        new EventTrigger("IntakeOut").onTrue(AutoCommands.intakeOut(intakeArm, intakeWheels));
        new EventTrigger("IntakeIn").onTrue(AutoCommands.intakeIn(intakeArm, intakeWheels));
        new EventTrigger("CollectPose").onTrue(AutoCommands.collectPose());

        //Configure autonomous selection
        boolean isCompetition = false;
        autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
      (stream) -> isCompetition
        ? stream.filter(auto -> auto.getName().startsWith("comp"))
        : stream
    );
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Manual single-action test commands (SmartDashboard + operator Back+D-pad
    }
    

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // Default: robot-centric drive (no gyro). Zero for settling period after enable.
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                double now = Timer.getFPGATimestamp();
                double sinceTeleop = now - s_teleopStartTimeSeconds;
                boolean settling = sinceTeleop < kDriveSettlingSeconds;

                double ly = safeJoystick(m_controller.getLeftY());
                double lx = safeJoystick(m_controller.getLeftX());
                double rx = safeJoystick(m_controller.getRightX());
                
                

                // Debug: see these on SmartDashboard when it spins (are requests zero? joystick noise?)
                SmartDashboard.putNumber("DriveDebug/TimeSinceTeleop", sinceTeleop);
                SmartDashboard.putBoolean("DriveDebug/Settling", settling);
                SmartDashboard.putNumber("DriveDebug/JoyLY", ly);
                SmartDashboard.putNumber("DriveDebug/JoyLX", lx);
                SmartDashboard.putNumber("DriveDebug/JoyRX", rx);

                if (settling) {
                    SmartDashboard.putNumber("DriveDebug/ReqVX", 0);
                    SmartDashboard.putNumber("DriveDebug/ReqVY", 0);
                    SmartDashboard.putNumber("DriveDebug/ReqRot", 0);
                    return s_idle;
                }

                // Radial deadband + cubic response: deadband on magnitude preserves diagonals,
                // cubing the magnitude gives fine control at low speed and full power at full deflection.
                double stickMag = Math.min(Math.hypot(lx, ly), 1.0);
                double dbMag = MathUtil.applyDeadband(stickMag, Deadband);
                double cubedMag = dbMag * dbMag * dbMag;
                double scale = stickMag > 1e-6 ? cubedMag / stickMag : 0;
                double vx = -ly * scale * MaxSpeed;
                double vy = -lx * scale * MaxSpeed;
                double rot = MathUtil.applyDeadband(-rx * MaxAngularRate, MaxAngularRate * RotDeadband);

                // While shooting (trigger held): reduce drive speed so feed is stable
                boolean shooting = RobotState.isShooting();
                if (shooting) {
                    vx = MathUtil.clamp(vx, -MaxSpeed*Constants.ShootDriveLimits.kSpeedScale, MaxSpeed * Constants.ShootDriveLimits.kSpeedScale);
                    vy = MathUtil.clamp(vy, -MaxSpeed*Constants.ShootDriveLimits.kSpeedScale, MaxSpeed * Constants.ShootDriveLimits.kSpeedScale);
                    rot = MathUtil.clamp(rot, -MaxAngularRate * Constants.ShootDriveLimits.kRotScale, MaxAngularRate * Constants.ShootDriveLimits.kRotScale);
                }
                m_prevVx = vx;
                m_prevVy = vy;
                m_prevRot = rot;

                SmartDashboard.putNumber("DriveDebug/ReqVX", vx);
                SmartDashboard.putNumber("DriveDebug/ReqVY", vy);
                SmartDashboard.putNumber("DriveDebug/ReqRot", rot);
                SmartDashboard.putBoolean("DriveDebug/ShootLimiting", shooting);
                return drivetrain.getFieldCentricDriveRequest(vx, vy, rot);
            })
            
        );

        Constants.Locations.addLocation("HUB_BLUE", Constants.FieldTarget.HUB_BLUE_X_M, Constants.FieldTarget.HUB_BLUE_Y_M, Constants.FieldTarget.HUB_BLUE_Z_M);       
        Constants.Locations.addLocation("HUB_RED", Constants.FieldTarget.HUB_RED_X_M, Constants.FieldTarget.HUB_RED_Y_M, Constants.FieldTarget.HUB_RED_Z_M);

        // ---- Intake: defaults read m_armState / m_intakeWheelsOn (set by path events or bumpers). ----
        intakeArm.setDefaultCommand(Commands.run(
            () -> {
                if (intakeArm.m_armState == 1) intakeArm.runExtend();
                else if (intakeArm.m_armState == 2) intakeArm.runRetract();
                else intakeArm.stop();
            }, intakeArm));
        intakeWheels.setDefaultCommand(Commands.run(
            () -> {
                if (intakeWheels.m_intakeWheelsOn) intakeWheels.runIn();
                else intakeWheels.stop();
            }, intakeWheels));

        // RB = toggle intake arm: one press = out (extend), next press = in (retract).
        m_controller.rightBumper().onTrue(
            Commands.runOnce(() -> intakeArm.m_armState = (intakeArm.m_armState == 1) ? 2 : 1));
        // LB = hold: wheels in + arm extend, release: stop wheels + arm retract (sets flags, default cmds run).
        m_controller.leftBumper()
            .onTrue(Commands.runOnce(() -> { intakeArm.m_armState = 1; intakeWheels.m_intakeWheelsOn = true; }))
            .onFalse(
                Commands.runOnce(() -> { intakeArm.m_armState = 2; intakeWheels.m_intakeWheelsOn = false; })
                .andThen(new WaitCommand(1.0f))
                .andThen(Commands.runOnce(() -> {intakeArm.stop();})));

        // Hood default: drive to manual target (operator D-pad adjusts this; defaults to zero).
        hood.setDefaultCommand(Commands.run(
            () -> hood.setPositionSetpoint(hood.getManualTarget()), hood));

    // TurretAim runs constantly: operator right/left stick or driver left trigger = aim, neither = hood at zero
    turret.setDefaultCommand(new TurretAim(drivetrain, turret, hood, flywheel, m_shotCalculator, m_operatorController, m_controller)
        .ignoringDisable(true));

    // ---- Operator (port 1): Back = stop all. Triggers/bumpers mirror driver exactly ----
    m_operatorController.back().onTrue(Commands.runOnce(this::stopAll));

    // Driver triggers/bumpers (same on both driver and operator):
    m_controller.rightTrigger().whileTrue(
        Commands.runOnce(() -> RobotState.setShootMode(frc.robot.RobotState.ShootMode.SHOOTING))
            .andThen(Commands.run(() -> {
              spindex.runForward();
              elevator.runReverse();
            }, spindex, elevator))
            .finallyDo(() -> {
              spindex.stop();
              elevator.stop();
              RobotState.setShootMode(frc.robot.RobotState.ShootMode.PREPARING);
            }));
    // Operator keeps trigger behavior (shoot) but DOES NOT mirror bumpers; bumpers are driver-only.
    m_operatorController.rightTrigger().whileTrue(
        Commands.runOnce(() -> RobotState.setShootMode(frc.robot.RobotState.ShootMode.SHOOTING))
            .andThen(Commands.run(() -> {
              spindex.runForward();
              elevator.runReverse();
            }, spindex, elevator))
            .finallyDo(() -> {
              spindex.stop();
              elevator.stop();
              RobotState.setShootMode(frc.robot.RobotState.ShootMode.PREPARING);
            }));
    // Operator face buttons: elevator (Y/X), spindex reverse (B), flywheel toggle (A)
    m_operatorController.y().whileTrue(Commands.run(elevator::runForward, elevator)).onFalse(Commands.runOnce(elevator::stop));
    m_operatorController.x().whileTrue(Commands.run(elevator::runReverse, elevator)).onFalse(Commands.runOnce(elevator::stop));
    m_operatorController.b().whileTrue(Commands.run(spindex::runReverse, spindex)).onFalse(Commands.runOnce(spindex::stop));
    // Flywheel manual: A = on/off toggle, RB = speed up, LB = speed down
    // No subsystem requirement so these don't interrupt TurretAim
    // Hood manual: D-pad Up = nudge hood up, D-pad Down = nudge hood down
    m_operatorController.povUp().onTrue(Commands.runOnce(hood::nudgeUp));
    m_operatorController.povDown().onTrue(Commands.runOnce(hood::nudgeDown));

        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> s_idle).ignoringDisable(true)
        );

        // Driver A = Spindex forward (while held)
        m_controller.a().whileTrue(Commands.run(spindex::runForward, spindex))
            .onFalse(Commands.runOnce(spindex::stop));
        // Driver B = point wheels at joystick direction
        m_controller.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-safeJoystick(m_controller.getLeftY()), -safeJoystick(m_controller.getLeftX())))
        ));
        // Driver Y = brake (hold)
        m_controller.y().whileTrue(drivetrain.applyRequest(() -> brake));

        // Left trigger = turret aim only (handled inside TurretAim via driver left trigger axis).

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        m_controller.back().and(m_controller.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        m_controller.back().and(m_controller.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        m_controller.start().and(m_controller.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        m_controller.start().and(m_controller.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Driver LEFT STICK CLICK = press to set forward (current heading becomes joystick forward).
        m_controller.leftStick().onTrue(drivetrain.runOnce(drivetrain::setForwardToCurrentHeading)); 
         

        drivetrain.registerTelemetry(logger::telemeterize);

        // OPERATOR BINDINGS — ADJUST: buttons/triggers here. Motor IDs/angles/RPM in Constants.java
        // Right trigger = full shoot (prepare then feed). Left trigger = pre-aim only (while held).
        // Left bumper = intake out, Right bumper = intake in. Start = climb mode. Y/A = climb (in climb mode).
        // TODO: Confirm operator port (1) and trigger thresholds (0.5); right = full shoot, left = pre-aim only.
        m_operatorController.start().onTrue(Commands.runOnce(() -> RobotState.setClimbMode(
                RobotState.getClimbMode() == frc.robot.RobotState.ClimbMode.ENABLED
                        ? frc.robot.RobotState.ClimbMode.DISABLED
                        : frc.robot.RobotState.ClimbMode.ENABLED)));
  }

  private void stopAll() {
    intakeWheels.stop();
    intakeArm.stopAndResetState();
    spindex.stop();
    elevator.stop();
    turret.stop();
    hood.stop();
  }


    /** Chassis speed magnitude (m/s) for intake wheel scaling. */
    private double getChassisSpeedMagnitude() {
        ChassisSpeeds s = drivetrain.getState().Speeds;
        return Math.hypot(s.vxMetersPerSecond, s.vyMetersPerSecond);
    }

    /** Clamp joystick to 0 if NaN/infinite (e.g. controller disconnected) to avoid erratic drive. */
    private static double safeJoystick(double value) {
        return Double.isFinite(value) ? value : 0.0;
    }

    /** Limit how fast a value can change per cycle (acceleration limiting). */
    private static double limitAccel(double target, double prev, double maxDelta) {
        double delta = target - prev;
        return prev + MathUtil.clamp(delta, -maxDelta, maxDelta);
    }

    /** Call from Robot.teleopPeriodic: for first kForceIdleSeconds after enable, force Idle to drivetrain (overrides command). Use to isolate spin: if it still spins, problem is not our commands. */
    public void forceDriveIdleForSettling() {
        if (kForceIdleSeconds <= 0) return;
        if (Timer.getFPGATimestamp() - s_teleopStartTimeSeconds < kForceIdleSeconds) {
            drivetrain.setControl(s_idle);
        }
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
