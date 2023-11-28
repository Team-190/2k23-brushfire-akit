package frc.robot.subsystems.extension;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ExtensionIOSim implements ExtensionIO {
  private ElevatorSim sim =
      new ElevatorSim(
          DCMotor.getFalcon500(1), 10.0, 7.0, Units.inchesToMeters(0.25), 0.3, 2.0, false, 0.3);
  private double appliedVolts;

  @Override
  public void updateInputs(ExtensionIOInputs inputs) {
    sim.update(0.02);
    inputs.positionRad = (sim.getPositionMeters()) / Units.inchesToMeters(0.25);
    inputs.velocityRadPerSec = sim.getVelocityMetersPerSecond() / Units.inchesToMeters(0.25);
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = new double[] {sim.getCurrentDrawAmps()};
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(appliedVolts);
  }
}
