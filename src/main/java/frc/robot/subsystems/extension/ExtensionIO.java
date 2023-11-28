package frc.robot.subsystems.extension;

import org.littletonrobotics.junction.AutoLog;

public interface ExtensionIO {
  @AutoLog
  public static class ExtensionIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
  }

  public default void updateInputs(ExtensionIOInputs inputs) {}

  public default void setVoltage(double volts) {}
}
