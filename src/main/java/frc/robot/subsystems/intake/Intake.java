package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;

public class Intake extends SubsystemBase {
    public enum Substate {
        STOPPED,
        PREPARING,
        READY
    }

    // declare IO & logs
    private final IntakeIO intakeIO;
    private @Getter Substate currentSubstate = Substate.STOPPED;
    private @Getter Substate desiredSubstate = Substate.STOPPED;
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
    
}
