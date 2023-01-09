// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static class DriveConstants {

        public static final double kDrivetrainTrackwidthMeters = 0.5461;
        public static final double kDrivetrainWheelbaseMeters = 0.5461;
        
        // angles in radians. 
        // to convert from degrees to radians multiply by pi/180 
        public static final double kFrontLeftSteerOffset = -.35;
        public static final double kFrontRightSteerOffset = .35;
        public static final double kBackLeftSteerOffset = .45;
        public static final double kBackRightSteerOffset = -.45;

        
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
            SdsModuleConfigurations.MK4_L1.getDriveReduction() *
            SdsModuleConfigurations.MK4_L1.getWheelDiameter() * Math.PI;


        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
        Math.hypot(kDrivetrainTrackwidthMeters / 2.0, kDrivetrainWheelbaseMeters / 2.0);
    }

}
