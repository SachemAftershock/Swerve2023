// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.lib.AftershockXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.commands.ManualDriveCommand;
import frc.robot.commands.RotateDriveCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private DriveSubsystem mDriveSubsystem = DriveSubsystem.getInstance();
  
  private final AftershockXboxController mControllerPrimary = new AftershockXboxController(0);
  private final Joystick mControllerSecondary = new Joystick(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    mDriveSubsystem.setDefaultCommand(new ManualDriveCommand(
            mDriveSubsystem,
            () -> -modifyAxis(mControllerPrimary.getLeftY()) * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(mControllerPrimary.getLeftX()) * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(mControllerSecondary.getTwist()) * DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.3
    ));
  }

  public void initialize() {
    mDriveSubsystem.initialize();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    TrajectoryConfig config = new TrajectoryConfig(
      DriveConstants.MAX_VELOCITY_METERS_PER_SECOND * 0.3, 
      DriveConstants.kMaxAccelerationMetersPerSecondSquared
    );

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(), 
      List.of(
        new Translation2d(1, 1), 
        new Translation2d(2, -1)
      ),
      new Pose2d(3, 0, new Rotation2d()), 
      config
    );

    return new FollowTrajectoryCommand(mDriveSubsystem, trajectory);
    //return new RotateDriveCommand(mDriveSubsystem, 90);


    //return new RotateDriveCommand(mDriveSubsystem, 90.0);

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

