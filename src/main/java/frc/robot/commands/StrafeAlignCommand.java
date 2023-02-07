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
    private final PID mStrafePidX;
    private final PID mStrafePidY;
    private boolean mIsFinished = false;

    public StrafeAlignCommand(DriveSubsystem drive, Limelight limelight) {
        mDrive = drive;
        mLimelight = limelight;
        mStrafePidX = new PID();
        mStrafePidY = new PID();
        addRequirements(mDrive);
    }

    @Override
    public void initialize() {

        System.out.println("Strafe Align Command starting");
        if (Math.abs(mLimelight.getTx()) > kLimelightOutOfBounds) {
            mIsFinished = true;
            return;
        }

        mDrive.drive(new ChassisSpeeds());
        mStrafePidX.start(kStrafeAlignGains);
        mStrafePidY.start(kDriveAngularGains);
    }

    @Override
    public void execute() {
        if (mIsFinished) return;

        //Checking to see if limelight is in range and we see the target
        if(!(mLimelight.getTy() > kMinimumDistanceFromTarget && mLimelight.getTy() < kMaximumDistanceFromTarget)) return;
        
        double speedX = mStrafePidX.update(mLimelight.getTx(), 0) * kMaxVelocityMetersPerSecond;  
        double speedY = -(mStrafePidY.update(mLimelight.getTy(), kMinimumDistanceFromTarget) *  kMaxVelocityMetersPerSecond);
    
        System.out.println("Speed X --> " + speedX + "Speed Y --> " + speedY);
        
        mDrive.drive(new ChassisSpeeds(speedY, speedX, 0));
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Strafe Align command ended");
        mDrive.drive(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(mStrafePidX.getError()) <= kStrafeAlignEpsilonX) 
            && (Math.abs(mStrafePidY.getError())) <= kStrafeAlignEpsilonY 
            || mIsFinished;
    }

}
