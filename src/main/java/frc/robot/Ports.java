package frc.robot;

import frc.robot.Ports.RioPortConstants.CanAddresses;

public class Ports {
    
    public static final class RioPortConstants {

        public static enum PwmPorts {
            ePwmPort0, ePwmPort1, ePwmPort2, ePwmPort3, ePwmPort4, ePwmPort5, ePwmPort6, ePwmPort7, ePwmPort8, ePwmPort9;
            public int get() { return ordinal(); }
        };

        public static enum DioPorts {
            eDioPort0, eDioPort1, eDioPort2, eDioPort3, eDioPort4, eDioPort5, eDioPort6, eDioPort7, eDioPort8, eDioPort9, 
            eDioPort10, eDioPort11, eDioPort12, eDioPort13, eDioPort14, eDioPort15, eDioPort16, eDioPort17, eDioPort18, eDioPort19;
            public int get() { return ordinal(); }
        };
        
        public static enum CanAddresses {
            eCanAddress0, eCanAddress1, eCanAddress2, eCanAddress3, eCanAddress4, eCanAddress5, eCanAddress6, eCanAddress7, eCanAddress8, eCanAddress9, 
            eCanAddress10, eCanAddress11, eCanAddress12, eCanAddress13, eCanAddress14, eCanAddress15, eCanAddress16, eCanAddress17, eCanAddress18;
            public int get() { return ordinal(); }
        };
    }

    public static final class DrivePorts {

        // Front Left
        public static final int kFrontLeftDriveMotorId = CanAddresses.eCanAddress15.get();
        public static final int kFrontLeftSteerMotorId = CanAddresses.eCanAddress14.get();
        public static final int kFrontLeftSteerEncoderId = CanAddresses.eCanAddress4.get();

        // Front Right
        public static final int kFrontRightDriveMotorId = CanAddresses.eCanAddress2.get();
        public static final int kFrontRightSteerMotorId = CanAddresses.eCanAddress1.get();
        public static final int kFrontRightSteerEncoderId = CanAddresses.eCanAddress10.get();

        // Back Left
        public static final int kBackLeftDriveMotorId = CanAddresses.eCanAddress6.get();
        public static final int kBackLeftSteerMotorId = CanAddresses.eCanAddress8.get();
        public static final int kBackLeftSteerEncoderId = CanAddresses.eCanAddress7.get();

        // Back Right
        public static final int kBackRightDriveMotorId = CanAddresses.eCanAddress5.get();
        public static final int kBackRightSteerMotorId = CanAddresses.eCanAddress11.get();
        public static final int kBackRightSteerEncoderId = CanAddresses.eCanAddress9.get();

    }

}
