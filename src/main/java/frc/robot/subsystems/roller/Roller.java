package frc.robot.subsystems.roller;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Roller extends SubsystemBase {
    private final RollerIO io;
    private final RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();
    
    public static final int INTAKE_MOTOR_ID = 60;
    public static final double IDLE_SPEED = 0.08;
    // why is this not 0?

    public static final double INTAKE_SPEED = 0.5;
    public static final double DUMP_SPEED = -0.25;
    public static final double HIGH_LAUNCH_SPEED = -1.0;
    public static final double MID_LAUNCH_SPEED = -1.0;
    public static final double HAS_CUBE_VELOCITY_THRESHOLD = Math.PI;

    public Roller(RollerIO io) {
        this.io = io;
        setDefaultCommand(run(this::idle));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        //Logger.processInputs("Intake", inputs);
    }

    public void setSpeedPercent(double motorSpeed){
        io.setVoltage(motorSpeed*12.0);
        
    }

    public void intake() {
        setSpeedPercent(INTAKE_SPEED);
    }

    public void outtake() {
        setSpeedPercent(MID_LAUNCH_SPEED);
    }

    public void idle() {
        setSpeedPercent(IDLE_SPEED);
    }
    
    public Command intakeCommand() {
        return startEnd(() -> intake(), () -> idle());
    }

    public Command outtakeCommand() {
        return startEnd(() -> outtake(), () -> idle());
    }
}
