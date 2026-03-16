package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.Setter;

public class Intake extends SubsystemBase {
    public enum Substate {
        STOPPED,
        ACTIVE
    }

    public Intake(IntakeIO intakeIO) {
        this.intakeIO = intakeIO;
        setArmEncoderPosition(0);
    }

    // declare IO & logs
    private @Getter final IntakeIO intakeIO;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    // private Substate previousSubstate = Substate.STOPPED;
    private @Getter Substate currentSubstate = Substate.STOPPED;
    private @Setter Substate desiredSubstate = Substate.STOPPED;

    private LoggedTunableNumber intakeSpeed = new LoggedTunableNumber("Intake/IntakeSpeed", 7);

    private Substate handleIntakeTransitions() {
        return desiredSubstate;
    }

    @Override
    public void periodic() {
        super.periodic();
        intakeIO.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
        Logger.recordOutput("Intake/CurrentSubstate", currentSubstate.toString());
        Logger.recordOutput("Intake/DesiredSubstate", desiredSubstate.toString());
        
        // previousSubstate = currentSubstate;
        currentSubstate = handleIntakeTransitions();
        applyStates();
    }

    public void applyStates() {
        switch (currentSubstate) {
            case STOPPED:
                setRollerVoltage(0);
                break;
            case ACTIVE:
                setRollerVoltage(intakeSpeed.get());
                break;
        }
    }

   
    public void setRollerVoltage(double voltage){
        intakeIO.setRollerVoltage(voltage);
    }
    public void setArmVoltage(double voltage){
        intakeIO.setArmVoltage(voltage);
    }

    public void moveArmToPosition(double positionRads){
        intakeIO.moveArmToPosition(positionRads);
    }
    public void setArmEncoderPosition(double positionRotations){
        intakeIO.setArmEncoderPosition(positionRotations);
    }
}
