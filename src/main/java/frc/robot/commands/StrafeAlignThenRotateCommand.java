package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.Limelight;
import frc.robot.subsystems.DriveSubsystem;

public class StrafeAlignThenRotateCommand extends SequentialCommandGroup {
    
    public StrafeAlignThenRotateCommand(DriveSubsystem drive, Limelight limelight) {
        addCommands(
            new RotateDriveCommand(drive, 0),
            new StrafeAlignCommand(drive, limelight),
            new RotateDriveCommand(drive, 0)
        );
    }

}
