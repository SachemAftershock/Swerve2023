package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.Limelight;
import frc.lib.PID;
import frc.robot.subsystems.DriveSubsystem;

import static frc.robot.Constants.DriveConstants.*;


public class StrafeAlignCommand extends CommandBase {
    
    private DriveSubsystem mDrive;
    private Limelight mLimelight;
    private final PID mStrafePid;
    private boolean mIsFinished = false;

    public StrafeAlignCommand(DriveSubsystem drive, Limelight limelight) {
        mDrive = drive;
        mLimelight = limelight;
        mStrafePid = new PID();
        addRequirements(mDrive);
    }


    @Override
    public void initialize() {
        if (Math.abs(mLimelight.getTx()) > kLimelightOutOfBounds) {
            mIsFinished = true;
            return;
        }

        mDrive.drive(new ChassisSpeeds());
        mStrafePid.start(kStrafeAlignGainse);
    }

    @Override
    public void execute() {
        if (mIsFinished) return;
        
        double speed = mStrafePid.update(mLimelight.getTx(), 0) 
        * kMaxVelocityMetersPerSecond;    
    
        mDrive.drive(new ChassisSpeeds(0, speed, 0));
    }

    @Override
    public void end(boolean interrupted) {
        mDrive.drive(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(mStrafePid.getError()) <= kStrafeAlignEpsilon) || mIsFinished;
    }

}
