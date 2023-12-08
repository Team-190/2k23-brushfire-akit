// Copyright 2021-2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.subsystems.bottompivot.BottomPivot;
import frc.robot.subsystems.bottompivot.BottomPivotIO;
import frc.robot.subsystems.bottompivot.BottomPivotIOSim;
import frc.robot.subsystems.bottompivot.BottomPivotIOTalonFX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.extension.Extension;
import frc.robot.subsystems.extension.ExtensionIOSim;
import frc.robot.subsystems.extension.ExtensionIOTalonFX;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerIOSim;
import frc.robot.subsystems.roller.RollerIOTalonFX;
import frc.robot.subsystems.toppivot.TopPivot;
import frc.robot.subsystems.toppivot.TopPivotIO;
import frc.robot.subsystems.toppivot.TopPivotIOSim;
import frc.robot.subsystems.toppivot.TopPivotIOTalonFX;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final BottomPivot bottomPivot;
  private final TopPivot topPivot;
  private final Roller roller;
  private final Extension extension;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // mechanism 2d
  @AutoLogOutput private final Mechanism2d mechanism = new Mechanism2d(2.0, 2.0);
  private final MechanismRoot2d mechanismRoot = mechanism.getRoot("Root", 1, 0.3);
  private final MechanismLigament2d bottomPivotLigament =
      mechanismRoot.append(
          new MechanismLigament2d("BottomPivot", 1.0, 90, 4, new Color8Bit(Color.kLightGreen)));
  private final MechanismLigament2d topPivotLigament =
      bottomPivotLigament.append(
          new MechanismLigament2d("TopPivot", .2, 90, 4, new Color8Bit(Color.kAliceBlue)));

  public void updateMechanism() {
    bottomPivotLigament.setAngle(bottomPivot.getPosition());
    topPivotLigament.setAngle(topPivot.getPosition());
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(0),
                new ModuleIOTalonFX(1),
                new ModuleIOTalonFX(2),
                new ModuleIOTalonFX(3));
        bottomPivot = new BottomPivot(new BottomPivotIOTalonFX());
        topPivot = new TopPivot(new TopPivotIOTalonFX());
        roller = new Roller(new RollerIOTalonFX());
        extension = new Extension(new ExtensionIOTalonFX());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        bottomPivot = new BottomPivot(new BottomPivotIOSim());
        topPivot = new TopPivot(new TopPivotIOSim());
        roller = new Roller(new RollerIOSim());
        extension = new Extension(new ExtensionIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        bottomPivot = new BottomPivot(new BottomPivotIO() {});
        topPivot = new TopPivot(new TopPivotIO() {});
        // roller = new Roller(new RollerIO() {});
        roller = new Roller(new RollerIOSim());
        extension = new Extension(new ExtensionIOSim());
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up FF characterization routines
    autoChooser.addOption(
        "Drive FF Characterization",
        new FeedForwardCharacterization(
            drive, drive::runCharacterizationVolts, drive::getCharacterizationVelocity));
    autoChooser.addOption("Test", topPivot.highLaunchCommand(true));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    controller
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
    controller.rightTrigger().whileTrue(roller.intakeCommand());
    controller.leftTrigger().whileTrue(roller.outtakeCommand());

    controller
        .y()
        .whileTrue(
            Commands.either(
                Commands.parallel(
                    bottomPivot.highLaunchCommand(true), topPivot.highLaunchCommand(true)),
                Commands.parallel(
                    bottomPivot.highLaunchCommand(false), topPivot.highLaunchCommand(false)),
                this::getFlipped));
    controller
        .b()
        .whileTrue(
            Commands.either(
                Commands.parallel(
                    bottomPivot.midLaunchCommand(true), topPivot.midLaunchCommand(true)),
                Commands.parallel(
                    bottomPivot.midLaunchCommand(false), topPivot.midLaunchCommand(false)),
                this::getFlipped));
    controller
        .a()
        .whileTrue(
            Commands.either(
                Commands.parallel(
                    bottomPivot.lowLaunchCommand(true), topPivot.lowLaunchCommand(true)),
                Commands.parallel(
                    bottomPivot.lowLaunchCommand(false), topPivot.lowLaunchCommand(false)),
                this::getFlipped));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  private boolean getFlipped() {
    double degrees = drive.getRotation().getDegrees();
    return Math.abs(degrees) > 90.0;
  }
}
