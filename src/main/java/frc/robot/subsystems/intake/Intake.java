package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.Setter;

public class Intake extends SubsystemBase {
    public enum Substate {
        STOPPED,
        ACTIVE
    }

    // declare IO & logs
    private final IntakeIO intakeIO;
    private @Getter Substate currentSubstate = Substate.STOPPED;
    private @Setter Substate desiredSubstate = Substate.STOPPED;
    private double intakeSpeed;

    public Intake(IntakeIO intakeIO) 
    {
        this.intakeIO = intakeIO;
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
    }

    public void applyStates() {
        switch (currentSubstate) {
            case STOPPED: intakeIO.setVelocity(0); 
            case ACTIVE: intakeIO.setVelocity(intakeSpeed);
        }
    }
    
}
