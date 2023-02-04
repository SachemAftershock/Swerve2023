package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.PID;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.TurnAngle;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ManualDriveCommandWithInstantTurn extends CommandBase {
    private final DriveSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;
    private final Joystick m_TurnAngle;

    private final PID mTurnPID;

    public ManualDriveCommandWithInstantTurn(DriveSubsystem drivetrainSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier,
                               Joystick turnAngle) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;
        this.m_TurnAngle = turnAngle;

        mTurnPID = new PID();

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        mTurnPID.start(DriveConstants.kDriveAngularGains);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        
        double rotationalSetpoint =  m_rotationSupplier.getAsDouble();
        int angle = m_TurnAngle.getPOV();
        
        if(angle != -1) {
            double current = m_drivetrainSubsystem.getYaw();
            rotationalSetpoint = mTurnPID.updateRotation(current, angle);
            System.out.println("PID OUT set to --> " + rotationalSetpoint + "current angle --> " + current + ": " + angle) ;
        }
        
        m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        m_translationXSupplier.getAsDouble(),
                        m_translationYSupplier.getAsDouble(),
                        rotationalSetpoint,
                        m_drivetrainSubsystem.getGyroscopeRotation()
                )
        );
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
