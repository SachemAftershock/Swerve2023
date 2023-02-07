package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.Limelight;
import frc.lib.PID;
import frc.robot.Robot;
import frc.robot.StateLogic;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveLocationLUT;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.StateLogic.RobotState;

public class DriveToSlotCommandContinous extends CommandBase {

    private DriveSubsystem mDrive;
    private RobotState mRobotState;
    private StateLogic mStateLogic;
    private DriveLocationLUT mDriveLocation;
    private double mXCoordSetpoint;
    private double mYCoordSetpoint;

    private PID mPIDX;
    private PID mPIDY;

    private boolean mIsFinished = false;

    public DriveToSlotCommandContinous(DriveSubsystem drive, StateLogic stateLogic, DriveLocationLUT driveLocation) {
        //Only passing in DriveLocationLUT enum for testing, should be using StateLogic to get RobotState object
        mDrive = drive;
        mStateLogic = stateLogic;
        mDriveLocation = driveLocation;

        mPIDX= new PID();
        mPIDY = new PID();

    }

    @Override
    public void initialize() {

        System.out.println("Drive to Slot command starting ");

        //mDriveLocation = mStateLogic.getRobotState().getDriveLocation();
        mXCoordSetpoint = mDriveLocation.getXCoord() - 0.5;
        mYCoordSetpoint = mDriveLocation.getYCoord() - 0.5;

        mPIDX.start(DriveConstants.kDriveToTargetGains);
        mPIDY.start(DriveConstants.kDriveToTargetGains);

        if((Math.abs(mDrive.getPose().getX())) < DriveConstants.kMinimumDistanceForAutoDrive 
            && Math.abs(mDrive.getPose().getY())  < DriveConstants.kMinimumDistanceForAutoDrive) {
            mIsFinished = true;
        }

    }

    @Override
    public void execute() {

        if(mIsFinished) return;

        //System.out.println("X --> " + mXCoordSetpoint);
        //System.out.println("Y --> " + mYCoordSetpoint);

        double xCurrent = mDrive.getPose().getX();
        double yCurrent = mDrive.getPose().getY();

        double xSpeed = mPIDX.update(xCurrent, mXCoordSetpoint);
        double ySpeed = mPIDY.update(yCurrent, mYCoordSetpoint);

        if(Math.abs(mPIDX.getError()) < DriveConstants.kDriveToTargetEpsilon) {
            xSpeed = 0.0;
        }

        if(Math.abs(mPIDY.getError()) < DriveConstants.kDriveToTargetEpsilon) {
            ySpeed = 0.0;
        }

        if(Math.abs(mPIDX.getError()) < DriveConstants.kDriveToTargetEpsilon
            && Math.abs(mPIDY.getError()) < DriveConstants.kDriveToTargetEpsilon) {
            return;
        }

        //System.out.println("X --> " + xCurrent + " Y--> " + yCurrent + " X speed --> " + xSpeed + " Y speed --> " + ySpeed);
        mDrive.drive(new ChassisSpeeds(xSpeed, ySpeed, 0));

    }

    @Override
    public void end(boolean interrupted) {
        mDrive.drive(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}



