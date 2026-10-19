package frc.robot.commands;

import java.lang.annotation.Target;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.constants.Location;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*Uses Limelight and drivetrain to see apriltags. We use apriltags to calculate where we are on the field and in
 * this method, we use the limelights to point the robot at the apriltags.
  */
public class AutoAimDriveUsingMegaTagLocations extends Command {

    private Location m_Targetlocation;
    public CommandSwerveDrivetrain m_Drivetrain;
    private DoubleSupplier Rotation;
    private DoubleSupplier JoyStickLeftY;
    private DoubleSupplier JoyStickLeftX;
    private double rotSpeedMultiplier = .15;
    private double baseSpeed = 0.1;
    private double Deadband = 1.5;
    private CommandXboxController joystick;
    private int FramesRan = 0;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
            .withDeadband(RobotContainer.MaxSpeed * 0.1).withRotationalDeadband(RobotContainer.MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    public AutoAimDriveUsingMegaTagLocations(Location loc, CommandSwerveDrivetrain drivetrain, CommandXboxController joystick)
    {
        //AutoAim drive also takes a "location" now, instead of a limelight.

        this.m_Drivetrain = drivetrain;
        m_Targetlocation = loc;
        
        this.joystick = joystick;
        addRequirements(m_Drivetrain);

    }

    @Override
    public void initialize(){
        double[] array = new double[3];
        array[0] = m_Targetlocation.spot.getX();
        array[1] = m_Targetlocation.spot.getY();
        array[2] = m_Targetlocation.spot.getZ();
        SmartDashboard.putString("LocationName", m_Targetlocation.Name);
        FramesRan = 0;
        JoyStickLeftX =  () -> { return joystick.getLeftX();};
        JoyStickLeftY =  () -> { return joystick.getLeftY();};
        Rotation = () -> {
            var state = m_Drivetrain.getState();
            var tx = m_Targetlocation.spot.getX();
            var ty = m_Targetlocation.spot.getY();
            var RobotPose = state.Pose;
            var RobotYaw = ((state.RawHeading.getDegrees()) % 360.0);
            SmartDashboard.putNumber("Raw Robot Yaw", state.RawHeading.getDegrees());
            SmartDashboard.putNumber("Robot Yaw", RobotYaw);

            //Lookat time
            var px = RobotPose.getX();
            var py = RobotPose.getY();

            var y = ty - py;
            var x = tx - px;
            var Theta = Math.atan2(y,x);

            Theta *= 180.0/Math.PI;

            SmartDashboard.putNumber("LookAtTheta", Theta);
                //replace the xdist grabber with the theta angle difference from the lookat angle to the selected target
                //m_Drivetrain.getState().Pose.get vector (robot x,y,z)
                //m_Drivetrain.getState().RawHeading.getDegrees();
            double xDist = Theta - RobotYaw;

            SmartDashboard.putNumber("Target Lookat Value diff", xDist);
            double maxXDist = 180.0; //Maximum lookat angle difference (180 degrees + -)
            double ratio = xDist / maxXDist;
            ratio = MathUtil.clamp(ratio, -1.0, 1.0);
            double rotSpeed = ratio * rotSpeedMultiplier;
            double realRotationSpeed = 0.0;
            if(ratio < 0)
                realRotationSpeed = rotSpeed - baseSpeed;
            else
                realRotationSpeed = rotSpeed + baseSpeed;
                
            return MathUtil.clamp(realRotationSpeed, -0.25, 0.25);
        };
    }
    
    @Override
    public void execute() 
    {     
        var Timer = new Timer();
        Timer.start();
        FramesRan += 1;
        ExecuteAlignment();
        Timer.stop();
        SmartDashboard.putNumber("LoopTime:", Timer.get());

    }

    public void ExecuteAlignment() {
                //SmartDashboard.putNumber("CurRotation:", Rotation.getAsDouble());
                //SmartDashboard.putNumber("CurFrame:", FramesRan);
                m_Drivetrain.setDefaultCommand(
                m_Drivetrain.applyRequest(() ->
                drive.withVelocityX(Math.pow(JoyStickLeftY.getAsDouble(), 3) * RobotContainer.MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(Math.pow(JoyStickLeftX.getAsDouble(),3) * RobotContainer.MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(Rotation.getAsDouble() * RobotContainer.MaxAngularRate) // Drive counterclockwise with negative X (left)
                    ));
    }

    @Override
    public boolean isFinished() 
    {
        return true;
    }
}
