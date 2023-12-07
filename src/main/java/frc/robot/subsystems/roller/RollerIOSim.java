package frc.robot.subsystems.roller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class RollerIOSim implements RollerIO {
  private final DCMotorSim motorSim;
  private double appliedVoltage;

  public RollerIOSim() {
    motorSim = new DCMotorSim(DCMotor.getFalcon500(1), 1, 0.02);
    appliedVoltage = 0.0;
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    motorSim.update(Constants.LOOP_PERIOD_SECS);

    inputs.positionRad = motorSim.getAngularPositionRad();
    inputs.velocityRadPerSec = motorSim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVoltage;
    inputs.currentAmps = new double[] {motorSim.getCurrentDrawAmps()};
  }

  @Override
  public void setVoltage(double volts) {
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12);
    motorSim.setInputVoltage(appliedVoltage);
  }
}
