package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import frc.lib.AftershockSubsystem;

import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Ports.DrivePorts.*;

public class DriveSubsystem extends AftershockSubsystem {
	private static DriveSubsystem mInstance;

	/**
	 * The maximum voltage that will be delivered to the drive motors.
	 * <p>
	 * This can be reduced to cap the robot's maximum speed. Typically, this is
	 * useful during initial testing of the robot.
	 */
	public static final double MAX_VOLTAGE = 12.0;
	// FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
	// The formula for calculating the theoretical maximum velocity is:
	// <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
	// pi
	// By default this value is setup for a Mk3 standard module using Falcon500s to
	// drive.
	// An example of this constant for a Mk4 L2 module with NEOs to drive is:
	// 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
	// SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
	/**
	 * The maximum velocity of the robot in meters per second.
	 * <p>
	 * This is a measure of how fast the robot should be able to drive in a straight
	 * line.
	 */
	// public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
	// SdsModuleConfigurations.MK3_STANDARD.getDriveReduction() *
	// SdsModuleConfigurations.MK3_STANDARD.getWheelDiameter() * Math.PI;
	/**
	 * The maximum angular velocity of the robot in radians per second.
	 * <p>
	 * This is a measure of how fast the robot can rotate in place.
	 */
	// Here we calculate the theoretical maximum angular velocity. You can also
	// replace this with a measured amount.
	// public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND =
	// MAX_VELOCITY_METERS_PER_SECOND /
	// Math.hypot(kDrivetrainTrackwidthMeters / 2.0, kDrivetrainWheelbaseMeters /
	// 2.0);

	private final SwerveDriveKinematics mKinematics = new SwerveDriveKinematics(
			// Front left
			new Translation2d(kDrivetrainTrackwidthMeters / 2.0, kDrivetrainWheelbaseMeters / 2.0),
			// Front right
			new Translation2d(kDrivetrainTrackwidthMeters / 2.0, -kDrivetrainWheelbaseMeters / 2.0),
			// Back left
			new Translation2d(-kDrivetrainTrackwidthMeters / 2.0, kDrivetrainWheelbaseMeters / 2.0),
			// Back right
			new Translation2d(-kDrivetrainTrackwidthMeters / 2.0, -kDrivetrainWheelbaseMeters / 2.0));

	private final SwerveDriveOdometry mOdometry = new SwerveDriveOdometry(
		mKinematics, 
		getGyroscopeRotation(),
		new Pose2d()
	);

	private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP

	private final SwerveModule mFrontLeftModule;
	private final SwerveModule mFrontRightModule;
	private final SwerveModule mBackLeftModule;
	private final SwerveModule mBackRightModule;

	private ChassisSpeeds mChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

	private DriveSubsystem() {
		ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

		mFrontLeftModule = Mk4SwerveModuleHelper.createFalcon500Neo(
				// This parameter is optional, but will allow you to see the current state of
				// the module on the dashboard.
				tab.getLayout("Front Left Module", BuiltInLayouts.kList)
						.withSize(2, 4)
						.withPosition(0, 0),
				// This can either be STANDARD or FAST depending on your gear configuration
				Mk4SwerveModuleHelper.GearRatio.L1,
				// This is the ID of the drive motor
				kFrontLeftDriveMotorId,
				// This is the ID of the steer motor
				kFrontLeftSteerMotorId,
				// This is the ID of the steer encoder
				kFrontLeftSteerEncoderId,
				// This is how much the steer encoder is offset from true zero (In our case,
				// zero is facing straight forward)
				kFrontLeftSteerOffset);

		mFrontRightModule = Mk4SwerveModuleHelper.createFalcon500Neo(
				tab.getLayout("Front Right Module", BuiltInLayouts.kList)
						.withSize(2, 4)
						.withPosition(2, 0),
				Mk4SwerveModuleHelper.GearRatio.L1,
				kFrontRightDriveMotorId,
				kFrontRightSteerMotorId,
				kFrontRightSteerEncoderId,
				kFrontRightSteerOffset);

		mBackLeftModule = Mk4SwerveModuleHelper.createFalcon500Neo(
				tab.getLayout("Back Left Module", BuiltInLayouts.kList)
						.withSize(2, 4)
						.withPosition(4, 0),
				Mk4SwerveModuleHelper.GearRatio.L1,
				kBackLeftDriveMotorId,
				kBackLeftSteerMotorId,
				kBackLeftSteerEncoderId,
				kBackLeftSteerOffset);

		mBackRightModule = Mk4SwerveModuleHelper.createFalcon500Neo(
				tab.getLayout("Back Right Module", BuiltInLayouts.kList)
						.withSize(2, 4)
						.withPosition(6, 0),
				Mk4SwerveModuleHelper.GearRatio.L1,
				kBackRightDriveMotorId,
				kBackRightSteerMotorId,
				kBackRightSteerEncoderId,
				kBackRightSteerOffset);

	}

	/**
	 * Sets the gyroscope angle to zero. This can be used to set the direction the
	 * robot is currently facing to the
	 * 'forwards' direction.
	 */
	public void zeroGyroscope() {
		m_navx.zeroYaw();
	}

	public Rotation2d getGyroscopeRotation() {
		if (m_navx.isMagnetometerCalibrated()) {
			// We will only get valid fused headings if the magnetometer is calibrated
			return Rotation2d.fromDegrees(m_navx.getFusedHeading());
		}

		// We have to invert the angle of the NavX so that rotating the robot
		// counter-clockwise makes the angle increase.
		return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
	}

	public void drive(ChassisSpeeds chassisSpeeds) {
		mChassisSpeeds = chassisSpeeds;
	}

	public void drive(SwerveModuleState[] states) {
		mChassisSpeeds = mKinematics.toChassisSpeeds(states);
	}

	@Override
	public void initialize() {
		mOdometry.resetPosition(new Pose2d(), new Rotation2d());
		zeroGyroscope();
	}

	@Override
	public void periodic() {
		mOdometry.update(getGyroscopeRotation(), 
			new SwerveModuleState(mFrontLeftModule.getDriveVelocity(), new Rotation2d(mFrontLeftModule.getSteerAngle())),
			new SwerveModuleState(mFrontRightModule.getDriveVelocity(), new Rotation2d(mFrontRightModule.getSteerAngle())),
			new SwerveModuleState(mBackLeftModule.getDriveVelocity(), new Rotation2d(mBackLeftModule.getSteerAngle())),
			new SwerveModuleState(mBackRightModule.getDriveVelocity(), new Rotation2d(mBackRightModule.getSteerAngle()))
		);

		SwerveModuleState[] states = mKinematics.toSwerveModuleStates(mChassisSpeeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

		mFrontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
				states[0].angle.getRadians());
		mFrontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
				states[1].angle.getRadians());
		mBackLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
				states[2].angle.getRadians());
		mBackRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
				states[3].angle.getRadians());
	}

	public Pose2d getPose() {
		return mOdometry.getPoseMeters();
	}

	public SwerveDriveKinematics getKinematics() {
		return mKinematics;
	}

	@Override
	public boolean checkSystem() {
		return true;
	}

	@Override
	public void outputTelemetry() {

	}

	public synchronized static DriveSubsystem getInstance() {
		if (mInstance == null) {
			mInstance = new DriveSubsystem();
		}
		return mInstance;
	}
}
