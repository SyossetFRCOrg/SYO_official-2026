package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.Setter;

// From last year with modifications (may need to implement a second motor)
public class Climber extends SubsystemBase {
    public enum Substate {
        STOPPED,
        UP,
        DOWN;
    }

    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
    private final ClimberIO climberIO;
    
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
                climberIO.setVoltage(0);
                break;
            case UP:
                climberIO.setVoltage(10);
                break;
            case DOWN:
                climberIO.setVoltage(-10);
                break;
        }
    }
}
