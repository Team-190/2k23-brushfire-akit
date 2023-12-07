package frc.robot.subsystems.roller;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class RollerIOTalonFX implements RollerIO {
  static final double UPDATE_FREQUENCY = 50.0;

  private final TalonFX motor;

  private final StatusSignal<Double> position;
  private final StatusSignal<Double> velocity;
  private final StatusSignal<Double> appliedVolts;
  private final StatusSignal<Double> current;

  public RollerIOTalonFX() {
    motor = new TalonFX(Roller.INTAKE_MOTOR_ID);

    TalonFXConfiguration configuration = new TalonFXConfiguration();

    configuration.CurrentLimits.SupplyCurrentLimit = 1.0;
    configuration.CurrentLimits.SupplyCurrentThreshold = 3.0;
    configuration.CurrentLimits.SupplyTimeThreshold = 0.04;
    configuration.CurrentLimits.SupplyCurrentLimitEnable = true;

    configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motor.getConfigurator().apply(configuration);

    position = motor.getPosition();
    velocity = motor.getVelocity();
    appliedVolts = motor.getMotorVoltage();
    current = motor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        UPDATE_FREQUENCY, position, velocity, appliedVolts, current);
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    BaseStatusSignal.refreshAll(position, velocity, appliedVolts, current);

    inputs.positionRad = Units.rotationsToRadians(position.getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = new double[] {current.getValueAsDouble()};
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(new VoltageOut(volts));
  }
}
