package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;


public class RotateDriveCommand extends CommandBase {

    private SwerveSubsystem mDrive;
    private double mAngularSetpoint;
    
    //Field Relative Rotation, downfield is 0deg
    public RotateDriveCommand(SwerveSubsystem drive, double setpoint) {
        mDrive = drive;
        mAngularSetpoint = setpoint;
        addRequirements(mDrive);
    }

    @Override
    public void initialize() {
        mDrive.setAutoRotate(mAngularSetpoint);
        System.out.println("RotateDriveCommand started " + Double.toString(mAngularSetpoint) + " degrees.");
    }

    @Override
    public void execute() {
        mDrive.runAutoRotate();
    }

    @Override
    public boolean isFinished() {
        if (mDrive.isAutoRotateFinished()) {
            System.out.println("RotateDriveCommand completed " + Double.toString(mAngularSetpoint) + " degrees.");
            return true;
        }
        return false;
    }
    @Override
    public void end(boolean interrupted) {
        mDrive.zeroHeading();
        mDrive.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}