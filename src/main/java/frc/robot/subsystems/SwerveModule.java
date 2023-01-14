package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.lib.TalonFxEncoder;

public class SwerveModule {

    private final WPI_TalonFX mDriveMotor; 
    private final CANSparkMax mTurningMotor;
    private TalonFxEncoder mDriveEncoder;
    private RelativeEncoder mTurningEncoder;

    private final PIDController turningPidController;

    private final AnalogInput absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new AnalogInput(absoluteEncoderId);

        mDriveMotor = new WPI_TalonFX(driveMotorId);
        mDriveMotor.configFactoryDefault();
        mDriveMotor.setNeutralMode(NeutralMode.Brake);
        mDriveMotor.configOpenloopRamp(0.5); //TODO: make a constant
        mDriveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 35, 37, 0.1));
        mDriveMotor.setInverted(driveMotorReversed); 

        //Make sure configure each spark to be brushless, since the setMotorType was depricated.
        mTurningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);
        mTurningMotor.restoreFactoryDefaults();
        mTurningMotor.setIdleMode(IdleMode.kBrake);
        mTurningMotor.setInverted(turningMotorReversed);
        mTurningMotor.burnFlash();

        mDriveEncoder = new TalonFxEncoder(mDriveMotor); //Does it need the speed or position
        mTurningEncoder = mTurningMotor.getEncoder();

        mDriveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        mDriveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        mTurningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        mTurningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getDrivePosition() {
        return mDriveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return mTurningEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return mTurningEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return mTurningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        mDriveEncoder.setPosition(0);
        mTurningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        mDriveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        mTurningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
    }

    public void stop() {
        mDriveMotor.set(0);
        mTurningMotor.set(0);
    }
}