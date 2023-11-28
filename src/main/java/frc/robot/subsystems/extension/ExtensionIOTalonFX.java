package frc.robot.subsystems.extension;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ExtensionIOTalonFX implements ExtensionIO {
  public static final double GEAR_RATIO = 10;
  private final TalonFX motor;

  private final StatusSignal<Double> position;
  private final StatusSignal<Double> velocity;
  private final StatusSignal<Double> appliedVolts;
  private final StatusSignal<Double> current;

  public ExtensionIOTalonFX() {
    motor = new TalonFX(0);
    TalonFXConfiguration configuration = new TalonFXConfiguration();
    // supply current limit
    configuration.CurrentLimits.SupplyCurrentLimit = 40.0;
    configuration.CurrentLimits.SupplyCurrentLimitEnable = true;

    // motor direction
    configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // set motor to brake mode
    configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // apply configuration to motor
    motor.getConfigurator().apply(configuration);

    position = motor.getPosition();
    velocity = motor.getVelocity();
    appliedVolts = motor.getMotorVoltage();
    current = motor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, velocity, appliedVolts, current);
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ExtensionIOInputs inputs) {}

  @Override
  public void setVoltage(double volts) {}
}
