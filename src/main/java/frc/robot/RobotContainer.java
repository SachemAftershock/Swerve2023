package frc.robot;

import java.util.List;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.AftershockXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
    private final AftershockXboxController mControllerPrimary = new AftershockXboxController(0);
    private JoystickButton mResetHeading;

    public RobotContainer() {
        // swerveSubsystem.setDefaultCommand(new SwerveJoystickCommand(
        //         swerveSubsystem,
        //         () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
        //         () -> driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
        //         () -> driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
        //         () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

        swerveSubsystem.setDefaultCommand(new SwerveJoystickCommand(
                swerveSubsystem, 
                () -> -modifyAxis(mControllerPrimary.getLeftY()) * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND,
                () -> -modifyAxis(mControllerPrimary.getLeftX()) * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND,
                () -> -modifyAxis(mControllerPrimary.getRightX()) * DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));        
        
        configureButtonBindings();

    }

    private void configureButtonBindings() {
        mResetHeading = new JoystickButton(mControllerPrimary, XboxController.Button.kA.value);
        mResetHeading.whenPressed(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
    }

    public void initialize() {

    }
    
    public Command getAutonomousCommand() {
        // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);

        // 2. Generate trajectory
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(1, 0),
                        new Translation2d(1, -1)),
                new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
                trajectoryConfig);

        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);

        // 5. Add some init and wrap-up, and return everything
        return new SequentialCommandGroup(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(() -> swerveSubsystem.stopModules()));
    }

    private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
          if (value > 0.0) {
            return (value - deadband) / (1.0 - deadband);
          } else {
            return (value + deadband) / (1.0 - deadband);
          }
        } else {
          return 0.0;
        }
      }
    
      private static double modifyAxis(double value) {
        // Deadband
        value = deadband(value, 0.15);
    
        // Square the axis
        value = Math.copySign(value * value, value);
    
        return value;
      }

}