// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


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

        public static enum CardinalDirection {
            eX, eY
        }

        public static enum TurnAngle {
            eForward(0), eLeft(90), eRight(-90), eBack(180), eDefault(999);

            private double mAngle; 

            private TurnAngle(double angle) {
                mAngle = angle;
            }

            public double getAngle() {
                return mAngle;
            }
            
        }

        public static final double kDriveControllerDeadband = 0.15;//0.05;
        public static final boolean kSquareAxis = false;

        public static final double[] kDriveAngularGains = {0.02, 0.0, 0.0}; //dont use I it sucks - Shreyas
        public static final double[] kDriveLinearGains = {0.4, 0.0, 0.0};
        public static final double[] kStrafeAlignGains = {0.025, 0, 0};
        public static final double[] kDriveToTargetGains = {0.50, 0, 0};
        public static final double[] kBalanceRobotGains = {0.0, 0.0, 0.0};

        public static final double kMaxStrafeVelocity = 5.0; //TODO: figure out an actual way to find the number

        public static final double kMinimumDistanceFromTarget = -18.37; //In degrees
        public static final double kMaximumDistanceFromTarget = -2.5;
        public static final double kMinimumDistanceForAutoDrive = 0.0; //TODO: find the this distance to avoid charged up station 

        public static final double kPX = 1.25;
        public static final double kPY = 1.25;

        public static final double kAutoRotateEpsilon = 3.0;
        public static final double kLinearDriveEpsilon = 0.0;
        public static final double kBalanceRobotEpsilon = 1.0;
        public static final double kDriveToTargetEpsilon = 0.10; //In meters
        
        
        public static final double kDrivetrainTrackwidthMeters = 0.5461;
        public static final double kDrivetrainWheelbaseMeters = 0.5461;
        
        // angles in radians. 
        // to convert from degrees to radians multiply by pi/180 
        public static final double kFrontLeftSteerOffset = -0.35 - (Math.PI / 2.0);//-.35;
        public static final double kFrontRightSteerOffset = 0.4 - (Math.PI / 2.0);//0.40;
        public static final double kBackLeftSteerOffset = 0.45 - (Math.PI / 2.0);//.45;
        public static final double kBackRightSteerOffset = -0.5 - (Math.PI / 2.0);//-.5;


        private static final double kMk4L1DriveReduction = (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0);
        private static final double kMk4WheelDiameter = 0.10033;

        
        public static final double kMaxVelocityMetersPerSecond = 6380.0 / 60.0 *
            kMk4L1DriveReduction * kMk4WheelDiameter * Math.PI;

        //TODO: Change
        public static final double kMaxAccelerationMetersPerSecondSquared = kMaxVelocityMetersPerSecond * 0.25;

        public static final double kMaxAngularVelocityRadiansPerSecond = kMaxVelocityMetersPerSecond /
        Math.hypot(kDrivetrainTrackwidthMeters / 2.0, kDrivetrainWheelbaseMeters / 2.0);

        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI;

        public static final double kLimelightOutOfBounds = 45;
        public static final double kStrafeAlignEpsilonX = 0.75;
        public static final double kStrafeAlignEpsilonY = 2.0;
    }

    public static enum ButtonState {
        eCone, eCube, eNone, eHigh, eMid, eFloor, eHumanStation, ePark, eIntake, eEject
    }

    public static enum DriveLocationLUT {

        eSlot1(7.2, -2.9), eSlot2(0.0, 0.0), eSlot3(0.0, 0.0),
        eSlot4(0.0, 0.0), eSlot5(0.0, 0.0), eSlot6(0.0, 0.0),
        eSlot7(0.0, 0.0), eSlot8(0.0, 0.0), eSlot9(0.0, 0.0), 
        
        eHumanStation(0.0, 0.0), eNone(0.0, 0.0);

        private double mXCoord;
        private double mYCoord;

        private DriveLocationLUT(double x, double y) {
            mXCoord = x;
            mYCoord = y;
        }

        public double getXCoord() {
            return mXCoord;
        }

        public double getYCoord() {
            return mYCoord;
        }

    }

    public static enum ElevatorStateLUT{

        eFloorCube(0.0,0.0),  
        eFloorCone(0.0,0.0),

        eHumanStationCone(0.0,0.0), 
        eHumanStationCube(0.0,0.0),

        eMidCone(0.0, 0.0),
        eMidCube(0.0, 0.0),

        eHighCone(0.0, 0.0),
        eHighCube(0.0, 0.0),
        
        ePark(0.0,0.0);

        private double mElevatorHeight;
        private double mArmExtension;

        private ElevatorStateLUT(double elevatorHeight, double armExtension) {
            mElevatorHeight = elevatorHeight;
            mArmExtension = armExtension;
        }

        public double getElevatorHeight() {
            return mElevatorHeight;
        }

        public double getArmExtension() {
            return mArmExtension;
        }

    }
}
