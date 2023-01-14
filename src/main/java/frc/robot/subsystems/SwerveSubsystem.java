package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.AftershockSubsystem;
import frc.lib.PID;
import frc.lib.Util;

import static frc.robot.Constants.DriveConstants.*;


public class SwerveSubsystem extends AftershockSubsystem {
    private final SwerveModule frontLeft = new SwerveModule(
            kFrontLeftDriveMotorPort,
            kFrontLeftTurningMotorPort,
            kFrontLeftDriveEncoderReversed,
            kFrontLeftTurningEncoderReversed,
            kFrontLeftDriveAbsoluteEncoderPort,
            kFrontLeftDriveAbsoluteEncoderOffsetRad,
            kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            kFrontRightDriveMotorPort,
            kFrontRightTurningMotorPort,
            kFrontRightDriveEncoderReversed,
            kFrontRightTurningEncoderReversed,
            kFrontRightDriveAbsoluteEncoderPort,
            kFrontRightDriveAbsoluteEncoderOffsetRad,
            kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            kBackLeftDriveMotorPort,
            kBackLeftTurningMotorPort,
            kBackLeftDriveEncoderReversed,
            kBackLeftTurningEncoderReversed,
            kBackLeftDriveAbsoluteEncoderPort,
            kBackLeftDriveAbsoluteEncoderOffsetRad,
            kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            kBackRightDriveMotorPort,
            kBackRightTurningMotorPort,
            kBackRightDriveEncoderReversed,
            kBackRightTurningEncoderReversed,
            kBackRightDriveAbsoluteEncoderPort,
            kBackRightDriveAbsoluteEncoderOffsetRad,
            kBackRightDriveAbsoluteEncoderReversed);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(kDriveKinematics, new Rotation2d(0));

    private PID mDriveLinearPID = new PID();
    private PID mDriveAngularPID = new PID();
    private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
          
    private double mAngularSetpoint; 
    private double mLinearSetpoint;

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    @Override
    public void initialize() {
        mAngularSetpoint = 0.0;
	    mLinearSetpoint = 0.0;
        
    }

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), frontLeft.getState(), frontRight.getState(), backLeft.getState(),
                backRight.getState());
        
    }

    public void setAutoRotate(double setpoint) {

		mAngularSetpoint = setpoint; //+ Util.normalizeAngle(getGyroscopeRotation().getDegrees());
		mDriveAngularPID.start(kDriveAngularGains);

		System.out.println("Starting auto rotate setpoint --> " + setpoint);
    }

    public void runAutoRotate() {

		double currentAngle = Util.normalizeAngle(gyro.getAngleAdjustment());
		double update = mDriveAngularPID.updateRotation(currentAngle, mAngularSetpoint) * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.5;
		m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, update);

		System.out.println("Running auto rotate angle --> " + currentAngle + " PID --> " + mDriveAngularPID.getError());

    }

    public boolean isAutoRotateFinished() {
	    return Math.abs(mDriveAngularPID.getError()) < kAutoRotateEpsilon;
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        m_chassisSpeeds = chassisSpeeds;
      }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(pose, getRotation2d());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    @Override
    public boolean checkSystem() {
		return true;
	}

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        
    }
}