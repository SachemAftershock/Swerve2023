package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import static frc.robot.Constants.DriveConstants.*;
import frc.robot.subsystems.DriveSubsystem;

public class FollowTrajectoryCommand extends CommandBase {

    private final DriveSubsystem mDrive;
    private final Trajectory mTrajectory;

    public FollowTrajectoryCommand(DriveSubsystem driveSubsystem, Trajectory trajectory) {
        mDrive = driveSubsystem;
        mTrajectory = trajectory;

        addRequirements(mDrive);
    }

    @Override
    public void initialize() {
        final double kP = kDriveAngularGains[0];
        final double kI = kDriveAngularGains[1];
        final double kD = kDriveAngularGains[2];

        final ProfiledPIDController thetaController =
        new ProfiledPIDController(
            kP, kI, kD, 
            new TrapezoidProfile.Constraints(
                MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                Math.PI //TODO: Change
            )
        );
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
        //Rotation is not preserved, may want to switch to other SwerveController constructor
        //and pass another function to preserve heading
        new SwerveControllerCommand(
            mTrajectory, 
            mDrive::getPose,
            mDrive.getKinematics(),
            new PIDController(kPX, 0, 0), new PIDController(kPY, 0, 0), thetaController, 
            mDrive::drive,
            mDrive).andThen(() -> mDrive.drive(new ChassisSpeeds())).schedule();
    }

    @Override
    public void end(boolean interrupted) {
        mDrive.drive(new ChassisSpeeds());
    }
}
