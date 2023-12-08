package frc.robot.subsystems.bottompivot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class BottomPivot extends SubsystemBase {
  public static final Rotation2d IDLE_POSITION = Rotation2d.fromDegrees(90.0);
  public static final Rotation2d HIGH_LAUNCH_POSITION = Rotation2d.fromDegrees(75);
  public static final Rotation2d MID_LAUNCH_POSITION = Rotation2d.fromDegrees(45);
  public static final Rotation2d LOW_LAUNCH_POSITION = Rotation2d.fromDegrees(25);

  public static final LoggedTunableNumber K_P = new LoggedTunableNumber("BottomPivot/kP", 0.2);
  public static final LoggedTunableNumber K_D = new LoggedTunableNumber("BottomPivot/kD", 0.001);
  public static final LoggedTunableNumber MAX_VELOCITY =
      new LoggedTunableNumber("BottomPivot/maxVelocity", 600.0);
  public static final LoggedTunableNumber MAX_ACCELERATION =
      new LoggedTunableNumber("BottomPivot/maxAcceleration", 650.0);

  private final BottomPivotIO io;
  private final BottomPivotIOInputsAutoLogged inputs = new BottomPivotIOInputsAutoLogged();

  private final ProfiledPIDController controller =
      new ProfiledPIDController(
          K_P.get(),
          0.0,
          K_D.get(),
          new TrapezoidProfile.Constraints(MAX_VELOCITY.get(), MAX_ACCELERATION.get()));

  private boolean closedLoop = true;

  // @AutoLogOutput private final Mechanism2d mechanism = new Mechanism2d(2.0,
  // 2.0);
  // private final MechanismRoot2d mechanismRoot = mechanism.getRoot("Root", 1,
  // 0.3);
  // private final MechanismLigament2d mechanismLigament =
  // mechanismRoot.append(
  // new MechanismLigament2d("BottomPivot", 1.2, 90, 4, new
  // Color8Bit(Color.kLightGreen)));

  public BottomPivot(BottomPivotIO io) {
    this.io = io;
    setDefaultCommand(run(() -> controller.setGoal(IDLE_POSITION.getDegrees())));
  }

  @AutoLogOutput
  public Rotation2d getPosition() {
    return new Rotation2d(inputs.positionRad);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("BottomPivot", inputs);

    // mechanismLigament.setAngle(getPosition());

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
      controller.reset(getPosition().getDegrees());
    } else if (closedLoop) {
      double voltage = controller.calculate(getPosition().getDegrees());
      io.setVoltage(voltage);
    }
  }

  @AutoLogOutput
  public boolean atGoal() {
    return controller.getSetpoint().equals(controller.getGoal());
  }

  public void setHighLaunch(boolean deployed, boolean flipped) {
    if (deployed) {
      if (flipped) {
        Rotation2d invertedAngle = Rotation2d.fromDegrees(180).minus(HIGH_LAUNCH_POSITION);
        controller.setGoal(invertedAngle.getDegrees());
      } else {
        controller.setGoal(HIGH_LAUNCH_POSITION.getDegrees());
      }
    } else {
      controller.setGoal(IDLE_POSITION.getDegrees());
    }
  }

  public void setMidLaunch(boolean deployed, boolean flipped) {
    if (deployed) {
      if (flipped) {
        Rotation2d invertedAngle = Rotation2d.fromDegrees(180).minus(MID_LAUNCH_POSITION);
        controller.setGoal(invertedAngle.getDegrees());
      } else {
        controller.setGoal(MID_LAUNCH_POSITION.getDegrees());
      }
    } else {
      controller.setGoal(IDLE_POSITION.getDegrees());
    }
  }

  public void setLowLaunch(boolean deployed, boolean flipped) {
    if (deployed) {
      if (flipped) {
        Rotation2d invertedAngle = Rotation2d.fromDegrees(180).minus(LOW_LAUNCH_POSITION);
        controller.setGoal(invertedAngle.getDegrees());
      } else {
        controller.setGoal(LOW_LAUNCH_POSITION.getDegrees());
      }
    } else {
      controller.setGoal(IDLE_POSITION.getDegrees());
    }
  }

  public Command lowLaunchCommand(boolean flipped) {
    return startEnd(() -> setLowLaunch(true, flipped), () -> setLowLaunch(false, flipped));
  }

  public Command midLaunchCommand(boolean flipped) {
    return startEnd(() -> setMidLaunch(true, flipped), () -> setMidLaunch(false, flipped));
  }

  public Command highLaunchCommand(boolean flipped) {
    return startEnd(() -> setHighLaunch(true, flipped), () -> setHighLaunch(false, flipped));
  }
}
