package frc.robot.subsystems.roller;

import com.ctre.phoenix6.hardware.TalonFX;

public class RollerIOTalonFX implements RollerIO {
    private final TalonFX motor;

    public RollerIOTalonFX() {
        motor = new TalonFX(Roller.INTAKE_MOTOR_ID);
    }

    @Override
    public void updateInputs(RollerIOInputs inputs) {
            
    }

    @Override
    public void setVoltage(double volts) {

    }
} 
