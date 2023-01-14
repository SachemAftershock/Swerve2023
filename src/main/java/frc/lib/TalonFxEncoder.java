package frc.lib;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

/**
 * Class to create an encoder object for Talon Fx motor controllers similar to Rev robotics
 * relative encoder class
 * 
 * (Class still under construction + not sure if needed)
 * 
 * @author Arhum Mudassir
 */
public class TalonFxEncoder {
   
    private TalonFX mTalon; 
    private double mTicksToMeters;
    private double mTicksToMetersPerSecond;

    /**
     * Constructor for TalonFcEncoder
     * @param talon TalonFX motor contrroller
     */
    public TalonFxEncoder(TalonFX mTalon) {
        this.mTalon = mTalon;
    }

    /**
     * Constant value for converting raw encoder ticks to meters
     * @param double
     */
    public void setPositionConversionFactor(double encoder2meters) {
        mTicksToMeters = encoder2meters;
    }

    /**
     * Constant value for converting raw encoder ticks to meters per second
     * @param double
     */
    public void setVelocityConversionFactor(double encoder2metersPerSecond) {
        mTicksToMetersPerSecond = encoder2metersPerSecond;
    }

    public double getPosition() {
        return 0.0;
    }

    public double getVelocity() {
        return 0.0;
    }

    public void setPosition(int position) {
        mTalon.setSelectedSensorPosition(0);
    }

}
