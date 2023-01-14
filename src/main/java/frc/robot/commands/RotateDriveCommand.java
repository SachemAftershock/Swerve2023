package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;


public class RotateDriveCommand extends CommandBase {

    private DriveSubsystem mDrive;
    private double mAngularSetpoint;
    private ChassisSpeeds m_chassisSpeeds;
    
    //Field Relative Rotation, downfield is 0deg
    public RotateDriveCommand(DriveSubsystem drive, double setpoint) {
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
        mDrive.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}