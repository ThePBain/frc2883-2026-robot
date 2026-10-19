package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/** Uses left joystick for drive, right for rotation. Field-centric with left bumper = set forward. */
public class RegularDrive extends Command {

    public CommandSwerveDrivetrain m_Drivetrain;
    private DoubleSupplier JoyStickLeftY;
    private DoubleSupplier JoyStickLeftX;
    private DoubleSupplier JoyStickRightX;
    private CommandXboxController joystick;

    public RegularDrive(CommandSwerveDrivetrain drivetrain, CommandXboxController joystick) {
        this.m_Drivetrain = drivetrain;
        this.joystick = joystick;
        addRequirements(m_Drivetrain);
    }

    @Override
    public void initialize() {
        JoyStickLeftX = () -> -joystick.getLeftX();
        JoyStickLeftY = () -> -joystick.getLeftY();
        JoyStickRightX = () -> -joystick.getRightX();
    }

    @Override
    public void execute() {
        ExecuteAlignment();
    }

    public void ExecuteAlignment() {
        m_Drivetrain.setDefaultCommand(m_Drivetrain.applyRequest(() -> {
            double vx = MathUtil.applyDeadband(Math.pow(JoyStickLeftY.getAsDouble(), 3) * RobotContainer.MaxSpeed, RobotContainer.MaxSpeed * 0.05);
            double vy = MathUtil.applyDeadband(Math.pow(JoyStickLeftX.getAsDouble(), 3) * RobotContainer.MaxSpeed, RobotContainer.MaxSpeed * 0.05);
            double rot = MathUtil.applyDeadband(JoyStickRightX.getAsDouble() * RobotContainer.MaxAngularRate, RobotContainer.MaxAngularRate * 0.05);
            return m_Drivetrain.getFieldCentricDriveRequest(vx, vy, rot);
        }));
    }

    @Override
    public boolean isFinished() 
    {
        return true;
    }
}
