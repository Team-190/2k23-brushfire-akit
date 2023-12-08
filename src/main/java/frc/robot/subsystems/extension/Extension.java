package frc.robot.subsystems.extension;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import frc.robot.util.LoggedTunableNumber;

public class Extension extends SubsystemBase {
  public static final Rotation2d IDLE_POSITION = Rotation2d.fromDegrees(0);
  public static final Rotation2d HIGH_LAUNCH_POSITION = Rotation2d.fromDegrees(-75);
  public static final Rotation2d MID_LAUNCH_POSITION = Rotation2d.fromDegrees(-45);
  public static final Rotation2d LOW_LAUNCH_POSITION = Rotation2d.fromDegrees(-25);
  public static final Rotation2d DEPLOYED_POSITION = Rotation2d.fromDegrees(120); // Intake Position

  public static final LoggedTunableNumber K_P = new LoggedTunableNumber("Extension/kP", 0.2);
  public static final LoggedTunableNumber K_D = new LoggedTunableNumber("Extension/kD", 0.001);

  private final ExtensionIO io;
  private final ExtensionIOInputsAutoLogged inputs = new ExtensionIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;

  public Extension(ExtensionIO io) {
    this.io = io;

    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(0.1, 0.05);
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(0.0, 0.03);
        break;
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        break;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Extension", inputs);
  }

  /** Run closed loop at the specified velocity. */
  public void runVelocity(double velocityRPM) {
    Logger.recordOutput("Extension/SetpointRPM", velocityRPM);
  }

  /** Stops the flywheel. */
  public void stop() {
    io.stop();
  }

  /** Returns the current velocity in RPM. */
  @AutoLogOutput
  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }

  /** Runs forwards at the commanded voltage. */
  public void runCharacterizationVolts(double volts) {
    io.setVoltage(volts);
  }

  /** Returns the average drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    return inputs.velocityRadPerSec;
  }
}
