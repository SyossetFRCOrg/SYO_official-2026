package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.Setter;

public class Shooter extends SubsystemBase {
    public enum Substate {
        STOPPED,
        PREPARING,
        ACTIVE
    }

    public Shooter(ShooterIO shooterIO) {
        this.shooterIO = shooterIO;
    }


    //Declare IO & logs
    private final ShooterIO shooterIO;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private static double shooterVoltage = 10, shooterChange = 0;


    private @Getter Substate currentSubstate = Substate.STOPPED;
    private @Setter Substate desiredSubstate = Substate.STOPPED;

    private final LoggedTunableNumber shootingEpsilon = new LoggedTunableNumber("Shooter/epsilon", 2);



    private Substate handleShooterTransitions() {
       return switch (desiredSubstate) {
            case STOPPED -> Substate.STOPPED;
            case PREPARING -> Substate.PREPARING;
            case ACTIVE -> motorsReady() ? Substate.ACTIVE : Substate.PREPARING;
        };
    }

    @Override
    public void periodic() {
        shooterIO.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        Logger.recordOutput("Shooter/CurrentSubstate", currentSubstate.toString());
        Logger.recordOutput("Shooter/DesiredSubstate", desiredSubstate.toString());
        Logger.recordOutput("Shooter/MotorsReady", motorsReady());
        Logger.recordOutput("Shooter/Difference", Math.abs(inputs.centerVelocityRadPerSec - (shooterVoltage + shooterChange) /*shooterVoltages.get(Substate.ACTIVE).get()*/));
        Logger.recordOutput("Shooter/Voltage", (currentSubstate == Substate.PREPARING ? 0.9 : 1) * shooterVoltage + shooterChange);
        Logger.recordOutput("Shooter/VoltageChange", shooterChange);
        currentSubstate = handleShooterTransitions();
        applyStates();
    }

    //TODO Update the setVoltage with linear regression
    public void applyStates() {
        switch (currentSubstate) {
            case STOPPED: 
                shooterIO.setVoltage(0);
                break;

            case ACTIVE:
                shooterIO.setVoltage(shooterVoltage + shooterChange);
                break; 
            case PREPARING:
                shooterIO.setVoltage(shooterVoltage * 0.9 + shooterChange);
                break;  
        }
    }


    //TODO
    public void setCalculatedShooterVoltage(double distance)
    {
        shooterVoltage = distance * 2; 
    }

    public boolean motorsReady()
    {
        return Math.abs(inputs.centerVelocityRadPerSec - (shooterVoltage + shooterChange) /*shooterVoltages.get(Substate.ACTIVE).get()*/) < shootingEpsilon.get();
    }

    public void adjustShooterVoltage(double amount) {
        shooterChange += amount;
    }

}
