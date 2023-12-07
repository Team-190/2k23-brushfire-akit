package frc.robot.subsystems.bottompivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class BottomPivotIOSim implements BottomPivotIO {
  private SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getFalcon500(1), 125.0, 1.5, 1.0, 0, Math.PI, false, Math.PI / 2);
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(BottomPivotIOInputs inputs) {
    sim.update(Constants.LOOP_PERIOD_SECS);

    inputs.positionRad = sim.getAngleRads();
    inputs.velocityRadPerSec = sim.getVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = new double[] {sim.getCurrentDrawAmps()};
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(appliedVolts);
  }
}
