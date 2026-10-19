# FRC Team 2883 – 2026 Competition Robot Code

Robot code for FIRST Robotics Competition Team 2883 (Warroad, MN), 2026 season, written in Java with WPILib. This robot won the **2026 Granite City Regional Autonomous Award** and placed **2nd at the Minnesota State Championship**.

I wrote this code as the team's President & Lead Programmer. Shared publicly with permission from the team.

## Highlights
- **Swerve drive:** Built on CTRE's Phoenix 6 swerve framework (base files generated with Tuner X), which I configured, tuned, and extended with vision fusion and autonomous path following.
- **Vision localization:** AprilTag detection on Limelight cameras using MegaTag pose estimates, fused with wheel odometry for field-relative localization (`subsystems/Limelight.java`, `RobotState.java`).
- **Auto-aim:** Automated aiming and drive alignment using the robot's field position (`commands/AutoAimDriveUsingMegaTagLocations.java`, `commands/TurretAim.java`, `vision/ShotCalculator.java`).
- **Autonomous routines:** Multiple starting-position autos built with PathPlanner (`src/main/deploy/pathplanner/`, `commands/AutoCommands.java`).
- **Mechanisms:** Subsystems for the turret, flywheel, hood, intake, spindexer, elevator, and LEDs (`subsystems/`).

## Project structure
- `src/main/java/frc/robot/subsystems/` – drivetrain, vision, and mechanism subsystems
- `src/main/java/frc/robot/commands/` – driver and autonomous commands
- `src/main/java/frc/robot/vision/` – shot calculation
- `src/main/deploy/pathplanner/` – autonomous paths and routines
- `*.md` notes – design notes on odometry, Limelight setup, and aim automation

## Built with
Java, WPILib, CTRE Phoenix 6, REVLib, PathPlanner, Limelight

## Credits
- `generated/TunerConstants.java` and the base of `CommandSwerveDrivetrain.java` were generated with CTRE Tuner X.
- `LimelightHelpers.java` is provided by Limelight.