package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Superstructure.SuperState;
import lombok.Getter;
import lombok.Setter;

public class Intake extends SubsystemBase {
    public enum Substate {
        STOPPED,
        ACTIVE
    }

    public Intake(IntakeIO intakeIO) {
        this.intakeIO = intakeIO;
    }

    // declare IO & logs
    private final IntakeIO intakeIO;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private Substate previousSubstate = Substate.STOPPED;
    private @Getter Substate currentSubstate = Substate.STOPPED;
    private @Setter Substate desiredSubstate = Substate.STOPPED;

    private double intakeSpeed = 5;

    private Substate handleIntakeTransitions() {
        return desiredSubstate;
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
        intakeIO.updateInputs(inputs);
        previousSubstate = currentSubstate;
        currentSubstate = handleIntakeTransitions();
        if (currentSubstate != previousSubstate) {
            System.out.println("Intake State Changed from " + previousSubstate + "to " + currentSubstate);
        }
        applyStates();
    }

    // TODO fix the setVelocity for stopped, 0 velocity seems to run the motor. Is
    // possibly a PID issue (?)
    public void applyStates() {
        switch (currentSubstate) {
            case STOPPED:
                intakeIO.setVelocity(0);
                break;
            case ACTIVE:
                intakeIO.setVelocity(intakeSpeed);
                break;
        }
    }
}
