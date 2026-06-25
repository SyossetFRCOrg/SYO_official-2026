package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.Setter;

public class Intake extends SubsystemBase {
    public enum Substate {
        STOPPED,
        ACTIVE,
        CLEANING
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
            case CLEANING:
                setRollerVoltage(intakeSpeed.get() * 0.4);
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
        intakeIO.moveArmToPosition(positionRads * (9.0/5.0));
    }
    public void setArmEncoderPosition(double positionRotations){
        intakeIO.setArmEncoderPosition(positionRotations);
    }
}
