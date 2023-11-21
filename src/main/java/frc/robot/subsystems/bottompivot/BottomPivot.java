package frc.robot.subsystems.bottompivot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class BottomPivot extends SubsystemBase {
    private final BottomPivotIO io;
    private final BottomPivotIOInputsAutoLogged inputs = new BottomPivotIOInputsAutoLogged();
    private final SimpleMotorFeedforward ffmodel;

    public BottomPivot(BottomPivotIO io) {
        this.io = io;

        switch (Constants.currentMode) {
            case REAL:
            case REPLAY:
                ffmodel = new SimpleMotorFeedforward(0.1,0.05);
                break;
            default:

        }
    }
} 