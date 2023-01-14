// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.fasterxml.jackson.databind.util.RootNameLookup;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.pidwrappers.PIDAnalogAccelerometer;
import frc.lib.AftershockSubsystem;
import frc.lib.PID;
import frc.lib.Util;

import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.Mk4ModuleConfiguration;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Ports.DrivePorts.*;

public class DriveSubsystem extends AftershockSubsystem {

		private static DriveSubsystem mInstance;

		double frontLeft = 0.0;
		double frontRight = 0;
		double backLeft = 0;
		double backRight = 0;
		int counter = 0;

  /**
   * The maximum voltage that will be delivered to the drive motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
   */
  public static final double MAX_VOLTAGE = 12.0;
  // FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
  //  The formula for calculating the theoretical maximum velocity is:
  //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
  //  By default this value is setup for a Mk3 standard module using Falcon500s to drive.
  //  An example of this constant for a Mk4 L2 module with NEOs to drive is:
  //   5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
  /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight line.
   */
//   public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
//           SdsModuleConfigurations.MK3_STANDARD.getDriveReduction() *
//           SdsModuleConfigurations.MK3_STANDARD.getWheelDiameter() * Math.PI;
  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
//   public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
//           Math.hypot(kDrivetrainTrackwidthMeters / 2.0, kDrivetrainWheelbaseMeters / 2.0);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
		  // Front left
		  new Translation2d(kDrivetrainTrackwidthMeters / 2.0, kDrivetrainWheelbaseMeters / 2.0),
		  // Front right
		  new Translation2d(kDrivetrainTrackwidthMeters / 2.0, -kDrivetrainWheelbaseMeters / 2.0),
		  // Back left
		  new Translation2d(-kDrivetrainTrackwidthMeters / 2.0, kDrivetrainWheelbaseMeters / 2.0),
		  // Back right
		  new Translation2d(-kDrivetrainTrackwidthMeters / 2.0, -kDrivetrainWheelbaseMeters / 2.0)
  );

  private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP

  private static DriveSubsystem mDriveSubsystem;

  private final SwerveModule mFrontLeftModule;
  private final SwerveModule mFrontRightModule;
  private final SwerveModule mBackLeftModule;
  private final SwerveModule mBackRightModule;

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  private PID mDriveLinearPID = new PID();
  private PID mDriveAngularPID = new PID();

  private double mAngularSetpoint; 
  private double mLinearSetpoint; 

  private DriveSubsystem() {
	ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

//     CANCoder mFrontLeftCanCoder = new CANCoder(kFrontLeftSteerEncoderId);
//     CANCoder mFrontRightCanCoder = new CANCoder(kFrontRightSteerEncoderId);
//     CANCoder mBacktLeftCanCoder = new CANCoder(kBackLeftSteerEncoderId);
//     CANCoder mBackRightCanCoder = new CANCoder(kBackRightSteerEncoderId);


	mFrontLeftModule = Mk4SwerveModuleHelper.createFalcon500Neo(
			// This parameter is optional, but will allow you to see the current state of the module on the dashboard.
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
			// This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
			kFrontLeftSteerOffset
	);

	mFrontRightModule = Mk4SwerveModuleHelper.createFalcon500Neo(
			tab.getLayout("Front Right Module", BuiltInLayouts.kList)
					.withSize(2, 4)
					.withPosition(2, 0),
			Mk4SwerveModuleHelper.GearRatio.L1,
			kFrontRightDriveMotorId,
			kFrontRightSteerMotorId,
			kFrontRightSteerEncoderId,
			kFrontRightSteerOffset
	);

	mBackLeftModule = Mk4SwerveModuleHelper.createFalcon500Neo(
			tab.getLayout("Back Left Module", BuiltInLayouts.kList)
					.withSize(2, 4)
					.withPosition(4, 0),
			Mk4SwerveModuleHelper.GearRatio.L1,
			kBackLeftDriveMotorId,
			kBackLeftSteerMotorId,
			kBackLeftSteerEncoderId,
			kBackLeftSteerOffset
	);

	mBackRightModule = Mk4SwerveModuleHelper.createFalcon500Neo(
			tab.getLayout("Back Right Module", BuiltInLayouts.kList)
					.withSize(2, 4)
					.withPosition(6, 0),
			Mk4SwerveModuleHelper.GearRatio.L1,
			kBackRightDriveMotorId,
			kBackRightSteerMotorId,
			kBackRightSteerEncoderId,
			kBackRightSteerOffset
	);

  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
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

   // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
   return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
	m_chassisSpeeds = chassisSpeeds;
  }

  @Override
  public void initialize() {

	mAngularSetpoint = 0.0;
	mLinearSetpoint = 0.0;

	zeroGyroscope();

  }


  @Override
  public void periodic() {
	SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
	SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

	mFrontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
	mFrontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
	mBackLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
	mBackRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
//     System.out.println("Front Left: " + mFrontLeftModule.getDriveVelocity());
//     System.out.println("Front Right: " + mFrontRightModule.getDriveVelocity());
//     System.out.println("Back Left: " + mBackLeftModule.getDriveVelocity());
//     System.out.println("Back Right: " + mBackRightModule.getDriveVelocity());
		frontLeft += mFrontLeftModule.getDriveVelocity();
		frontRight += mFrontRightModule.getDriveVelocity();
		backLeft += mBackLeftModule.getDriveVelocity();
		backRight += mBackRightModule.getDriveVelocity();

		if(counter == 100) {
				// System.out.println("Front Left: " + frontLeft);
				// System.out.println("Front Right: " + frontRight);
				// System.out.println("Back Left: " + backLeft);
				// System.out.println("Back Right: " + backRight);
				counter = 0;

				System.out.println("GYRO ANGLE --> " + m_navx.getAngle());
		}
		counter++;


		
  }

  public void setAutoRotate(double setpoint) {

		mAngularSetpoint = setpoint; //+ Util.normalizeAngle(getGyroscopeRotation().getDegrees());
		mDriveAngularPID.start(kDriveAngularGains);

		System.out.println("Starting auto rotate setpoint --> " + setpoint);

  }

  public void runAutoRotate() {

		double currentAngle = Util.normalizeAngle(getGyroscopeRotation().getDegrees());
		double update = mDriveAngularPID.updateRotation(currentAngle, mAngularSetpoint) * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.5;
		m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, update);

		System.out.println("Running auto rotate angle --> " + currentAngle + " PID --> " + mDriveAngularPID.getError());

  }

  public boolean isAutoRotateFinished() {
	  return Math.abs(mDriveAngularPID.getError()) < kAutoRotateEpsilon;
  }

  @Override
  public boolean checkSystem() {
		return true;
	}
  

//   public static DriveSuUbsystem getInstance() {
//         if (mDriveSubsystem == null) mDriveSubsystem = new DriveSubsystem();
//         return mDriveSubsystem;
//   }

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
