package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ToggleableSubsystem;
import lombok.Getter;
import lombok.Setter;

// From last year with modifications (may need to implement a second motor)
public class Climber extends ToggleableSubsystem {
    public enum Substate {
        STOPPED,
        L1, // Bottom rung
        L2, // Middle rung
        L3; // Top rung
    }

    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
    private final ClimberIO climberIO;
    
    private double voltage = 0;
    private @Getter Substate currentSubstate = Substate.STOPPED;
    private @Setter Substate desiredSubstate = Substate.STOPPED;

    public Climber(ClimberIO climberIO) {
        this.climberIO = climberIO;
    }

    @Override
    public void periodic() {
        super.periodic();
        currentSubstate = handleClimberTransitions();
        climberIO.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
        applyStates();
    }

    private Substate handleClimberTransitions() {
        return desiredSubstate;
    }

    //TODO actually add logic
    public void applyStates() {
        switch (currentSubstate) {
            case STOPPED:
                voltage = 0;
                climberIO.setvoltage(voltage);
                break;
            case L1:
                /*
                Example barebones idea if two motors:
                voltage1 = 1;
                voltage2 = 0 or -1 (to pull back down);
                */
                break;
            case L2:
                break;
            case L3:
                break;
        }
    }
}
