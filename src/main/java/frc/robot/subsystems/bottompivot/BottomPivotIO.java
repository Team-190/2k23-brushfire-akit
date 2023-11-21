package frc.robot.subsystems.bottompivot;

import org.littletonrobotics.junction.AutoLog;

public interface BottomPivotIO {
    @AutoLog    //creates methods like tolog and fromlog automatically
    public static class BottomPivotIOInputs {
        public double positionRad = 0.0;   
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double[] currentAmps = new double[] {};
    }

     /** Updates the set of loggable inputs. */
    public default void updateInputs(BottomPivotIOInputs inputs) {}

    /** Run open loop at the specified voltage. */
    public default void setVoltage(double volts) {}

    /** Stop in open loop. */
    public default void stop() {}; 
}

