package frc.robot.subsystems.toppivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class TopPivotIOSim implements TopPivotIO {
  private SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getFalcon500(1), 125, 1.5, .5, -Math.PI / 2, Math.PI / 2, false, 0);
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(TopPivotIOInputs inputs) {
    sim.update(Constants.LOOP_PERIOD_SECS);

    inputs.positionRad = sim.getAngleRads();
    inputs.velocityRadPerSec = sim.getVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = new double[] {sim.getCurrentDrawAmps()};
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12, 12);
    sim.setInputVoltage(appliedVolts);
  }
}
