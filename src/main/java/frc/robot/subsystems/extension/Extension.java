package frc.robot.subsystems.extension;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Extension extends SubsystemBase {
  // public static final Rotation2d IDLE_POSITION = Rotation2d.fromDegrees(0);
  // public static final Rotation2d HIGH_LAUNCH_POSITION = Rotation2d.fromDegrees(-75);
  // public static final Rotation2d MID_LAUNCH_POSITION = Rotation2d.fromDegrees(-45);
  // public static final Rotation2d LOW_LAUNCH_POSITION = Rotation2d.fromDegrees(-25);
  // public static final Rotation2d DEPLOYED_POSITION = Rotation2d.fromDegrees(120); // Intake
  // Position

  // all units in meters
  public static final double IDLE_POSITION = 0.3;
  public static final double HIGH_LAUNCH_POSITION = 2;
  public static final double MID_LAUNCH_POSITION = 1.5;
  public static final double LOW_LAUNCH_POSITION = 1;
  public static final double DEPLOYED_POSITION = 0.5;

  public static final LoggedTunableNumber K_P = new LoggedTunableNumber("Extension/kP", 0.2);
  public static final LoggedTunableNumber K_D = new LoggedTunableNumber("Extension/kD", 0.001);

  public static final LoggedTunableNumber MAX_VELOCITY =
      new LoggedTunableNumber("Extension/maxVelocity", 600.0);
  public static final LoggedTunableNumber MAX_ACCELERATION =
      new LoggedTunableNumber("Extension/maxAcceleration", 650.0);

  private final ExtensionIO io;
  private final ExtensionIOInputsAutoLogged inputs = new ExtensionIOInputsAutoLogged();

  private final ProfiledPIDController controller =
      new ProfiledPIDController(
          K_P.get(),
          0.0,
          K_D.get(),
          new TrapezoidProfile.Constraints(MAX_VELOCITY.get(), MAX_ACCELERATION.get()));

  private boolean closedLoop = true;

  public Extension(ExtensionIO io) {
    this.io = io;
    setDefaultCommand(run(() -> controller.setGoal(IDLE_POSITION)));
  }

  @AutoLogOutput
  public double getPosition() {
    return (inputs.positionRad) * Units.inchesToMeters(0.25); // 0.25 is the drum radius
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Extension", inputs);

    if (K_P.hasChanged(hashCode())) {
      controller.setP(K_P.get());
    }
    if (K_D.hasChanged(hashCode())) {
      controller.setD(K_D.get());
    }
    if (MAX_VELOCITY.hasChanged(hashCode()) || MAX_ACCELERATION.hasChanged(hashCode())) {
      controller.setConstraints(
          new TrapezoidProfile.Constraints(MAX_VELOCITY.get(), MAX_ACCELERATION.get()));
    }

    if (DriverStation.isDisabled()) {
      controller.reset(getPosition());
    } else if (closedLoop) {
      double voltage = controller.calculate(getPosition());
      io.setVoltage(voltage);
    }

    Logger.recordOutput("Extension/setpoint", controller.getSetpoint().position);
  }

  @AutoLogOutput
  public boolean atGoal() {
    return controller.getSetpoint().equals(controller.getGoal());
  }

  public void setHighLaunch(boolean deployed) {
    if (deployed) {
      controller.setGoal(HIGH_LAUNCH_POSITION);

    } else {
      controller.setGoal(IDLE_POSITION);
    }
  }

  public void setMidLaunch(boolean deployed) {
    if (deployed) {
      controller.setGoal(MID_LAUNCH_POSITION);
    } else {
      controller.setGoal(IDLE_POSITION);
    }
  }

  public void setLowLaunch(boolean deployed) {
    if (deployed) {
      controller.setGoal(LOW_LAUNCH_POSITION);
    } else {
      controller.setGoal(IDLE_POSITION);
    }
  }

  public Command lowLaunchCommand(boolean flipped) {
    return startEnd(() -> setLowLaunch(true), () -> setLowLaunch(false));
  }

  public Command midLaunchCommand(boolean flipped) {
    return startEnd(() -> setMidLaunch(true), () -> setMidLaunch(false));
  }

  public Command highLaunchCommand(boolean flipped) {
    return startEnd(() -> setHighLaunch(true), () -> setHighLaunch(false));
  }
}
