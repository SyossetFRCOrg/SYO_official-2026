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
        ACTIVE,
        FERRY
    }

    public Shooter(ShooterIO shooterIO) {
        this.shooterIO = shooterIO;
    }


    //Declare IO & logs
    private final ShooterIO shooterIO;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private static double shooterVelocity = 70, shooterChange = 0;


    private @Getter Substate currentSubstate = Substate.STOPPED;
    private @Setter Substate desiredSubstate = Substate.STOPPED;

    private final LoggedTunableNumber shootingEpsilon = new LoggedTunableNumber("Shooter/epsilon", 2);
 


    private Substate handleShooterTransitions() {
       return switch (desiredSubstate) {
            case STOPPED -> Substate.STOPPED;
            case PREPARING -> Substate.PREPARING;
            case ACTIVE -> motorsReady() ? Substate.ACTIVE : Substate.PREPARING;
            case FERRY -> Substate.FERRY;
        };
    }

    @Override
    public void periodic() {
        shooterIO.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        Logger.recordOutput("Shooter/CurrentSubstate", currentSubstate.toString());
        Logger.recordOutput("Shooter/DesiredSubstate", desiredSubstate.toString());
        Logger.recordOutput("Shooter/MotorsReady", motorsReady());
        Logger.recordOutput("Shooter/Difference", Math.abs(inputs.centerVelocityRotPerSec - (shooterVelocity + shooterChange) /*shooterVoltages.get(Substate.ACTIVE).get()*/));
        Logger.recordOutput("Shooter/InputtedVelocity",  shooterVelocity + shooterChange);
        Logger.recordOutput("Shooter/VelocityChange", shooterChange);
        currentSubstate = handleShooterTransitions();
        applyStates();
    }

    //TODO Update the setVoltage with linear regression
    public void applyStates() {
        switch (currentSubstate) {
            case STOPPED: 
                shooterIO.setVelocityVoltage(0);
                break;
            case ACTIVE, PREPARING, FERRY:
                shooterIO.setVelocityVoltage(shooterVelocity + shooterChange);
                break;
            }
    }

    public void setCalculatedShooterVoltage(double distance)
    {
        if(currentSubstate == Substate.ACTIVE || currentSubstate == Substate.PREPARING)
        {
            //shooterVelocity = ShooterConstants.shooterSpeedMapScoring.get(distance);
        }
        else if(currentSubstate == Substate.FERRY)
        {
           // shooterVelocity = ShooterConstants.shooterSpeedMapFerrying.get(distance);
        }
    }

    public boolean motorsReady()
    {
        return Math.abs(inputs.centerVelocityRotPerSec - (shooterVelocity + shooterChange) /*shooterVoltages.get(Substate.ACTIVE).get()*/) < shootingEpsilon.get();
    }

    public void adjustShooterVoltage(double amount) {
        shooterChange += amount;
    }
}
