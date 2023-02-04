// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import javax.sound.sampled.LineEvent;

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
import frc.lib.Limelight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.CardinalDirection;
import frc.robot.Constants.DriveConstants.TurnAngle;
import frc.robot.commands.FollowTrajectoryCommandFactory;
import frc.robot.commands.LinearDriveCommand;
import frc.robot.commands.ManualDriveCommand;
import frc.robot.commands.ManualDriveCommandWithInstantTurn;
import frc.robot.commands.StrafeAlignCommand;
import frc.robot.commands.StrafeAlignCommandContinous;
import frc.robot.commands.StrafeAlignThenRotateCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private DriveSubsystem mDriveSubsystem = DriveSubsystem.getInstance();
  
  private final Joystick mControllerPrimary = new Joystick(0);
  private final Joystick mControllerSecondary = new Joystick(1);

  private JoystickButton mStrafeAutoAlign; 
  private static TurnAngle mTurnAngle;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
	  // Configure the button bindings
    configureButtonBindings();
    // mDriveSubsystem.setDefaultCommand(new ManualDriveCommand(
    //         mDriveSubsystem,
    //         () -> -modifyAxis(mControllerPrimary.getY()) * DriveConstants.kMaxVelocityMetersPerSecond,
    //         () -> -modifyAxis(mControllerPrimary.getX()) * DriveConstants.kMaxVelocityMetersPerSecond,
    //         () -> -modifyAxis(mControllerSecondary.getTwist()) * DriveConstants.kMaxAngularVelocityRadiansPerSecond * 0.3
    // ));

    mTurnAngle = TurnAngle.eDefault;

    mDriveSubsystem.setDefaultCommand(new ManualDriveCommandWithInstantTurn(
    			mDriveSubsystem, 
				() -> -modifyAxis(mControllerPrimary.getY()) * DriveConstants.kMaxVelocityMetersPerSecond,
				() -> -modifyAxis(mControllerPrimary.getX()) * DriveConstants.kMaxVelocityMetersPerSecond,
				() -> -modifyAxis(mControllerSecondary.getTwist()) * DriveConstants.kMaxAngularVelocityRadiansPerSecond * 0.3, 
				mControllerSecondary
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
  private void configureButtonBindings() {

	  mStrafeAutoAlign = new JoystickButton(mControllerPrimary, Joystick.ButtonType.kTrigger.value);
    mStrafeAutoAlign.whileTrue(new StrafeAlignCommandContinous(mDriveSubsystem, mDriveSubsystem.getLimelight()));

  }

  public void periodic() {

    //System.out.println(mControllerSecondary.getPOV());

    if(mControllerSecondary.getPOV() == -1) {
      setTurnAngle(TurnAngle.eDefault);
    }

    if(mControllerSecondary.getPOV() == 0) {
      setTurnAngle(TurnAngle.eForward);
    }

    if(mControllerSecondary.getPOV() == 90) {
      setTurnAngle(TurnAngle.eRight);
    }

    if(mControllerSecondary.getPOV() == 180) {
      setTurnAngle(TurnAngle.eBack);
    }

    if(mControllerSecondary.getPOV() == 270) {
      setTurnAngle(TurnAngle.eLeft);
    }

  }

  private static void setTurnAngle(TurnAngle angle) {
	  mTurnAngle = angle;
  }

  private static TurnAngle getTurnAngle() {
	  return mTurnAngle;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {


    TrajectoryConfig config = new TrajectoryConfig(
      DriveConstants.kMaxVelocityMetersPerSecond * 0.3, 
      DriveConstants.kMaxAccelerationMetersPerSecondSquared
    );

    // Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    //   new Pose2d(), 
    //   List.of(
    //     new Translation2d(2.0, -0.5)
    //     //new Translation2d(0, 1.2)//,
    //     //new Translation2d(1, 1.5)
    //     //new Translation2d(2.2,0)
    //   ),
    //   new Pose2d(1.0, -0.4, new Rotation2d()), 
    //   config
    // );

    // Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(
    //   new Pose2d(), 
    //   List.of( 
    //     new Translation2d(0.5, 1.5),
    //     new Translation2d(1.0, 1.5),
    //     new Translation2d(1.5, 1.5)
    //   ),
    //   new Pose2d(1.5, 1.5, new Rotation2d()),
    //   config
    // );


    Trajectory pathOne = TrajectoryGenerator.generateTrajectory(
      new Pose2d(1.0, 3.0, new Rotation2d()), 
      List.of(
        new Translation2d(2.0, -0.5)
        //new Translation2d(0, 1.2)//,
        //new Translation2d(1, 1.5)
        //new Translation2d(2.2,0)
      ),
      new Pose2d(1.0, -0.4, new Rotation2d()), 
      config
    );

    //return new LinearDriveCommand(mDriveSubsystem, 2.0, CardinalDirection.eX);
    //return new StrafeAlignThenRotateCommand(mDriveSubsystem, mDriveSubsystem.getLimelight());

    return new StrafeAlignCommand(mDriveSubsystem, mDriveSubsystem.getLimelight());

    // return new SequentialCommandGroup(
    //   //FollowTrajectoryCommandFactory.generateCommand(mDriveSubsystem, pathOne)
    //   //new RotateDriveCommand(mDriveSubsystem, 90),
    //   //FollowTrajectoryCommandFactory.generateCommand(mDriveSubsystem, trajectory2)
    //   //new RotateDriveCommand(mDriveSubsystem, -30)
    // );
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
    value = deadband(value, DriveConstants.kDriveControllerDeadband);

    // Square the axis
    if(DriveConstants.kSquareAxis) {
      value = Math.copySign(value * value, value);
    }

    return value;
  }
}

