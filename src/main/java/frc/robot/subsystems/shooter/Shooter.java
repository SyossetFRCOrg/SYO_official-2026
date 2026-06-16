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
        CLEANING
    }

    public Shooter(ShooterIO shooterIO) {
        this.shooterIO = shooterIO;
    }


    //Declare IO & logs
    private final ShooterIO shooterIO;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private static double shooterVelocity = 70, shooterChange = 5;


    private @Getter Substate currentSubstate = Substate.STOPPED;
    private @Setter Substate desiredSubstate = Substate.STOPPED;

    private final LoggedTunableNumber shootingEpsilon = new LoggedTunableNumber("Shooter/epsilon", 2);
    private final LoggedTunableNumber shooterCheck = new LoggedTunableNumber("Shooter/check", 2);

    @SuppressWarnings("unused")
    private boolean isFerry = false;
 


    private Substate handleShooterTransitions() {
       return switch (desiredSubstate) {
            case STOPPED -> Substate.STOPPED;
            case PREPARING -> Substate.PREPARING;
            case ACTIVE -> motorsReady() || currentSubstate == Substate.ACTIVE ? Substate.ACTIVE : Substate.PREPARING;
            case CLEANING -> Substate.CLEANING;
        };
    }

    @Override
    public void periodic() {
        shooterIO.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        Logger.recordOutput("Shooter/CurrentSubstate", currentSubstate.toString());
        Logger.recordOutput("Shooter/DesiredSubstate", desiredSubstate.toString());
        Logger.recordOutput("Shooter/MotorsReady", motorsReady());
        Logger.recordOutput("Shooter/Difference", Math.abs(inputs.leftVelocityRotPerSec - (shooterVelocity + shooterChange) /*shooterVoltages.get(Substate.ACTIVE).get()*/));
        Logger.recordOutput("Shooter/InputtedVelocity",  shooterVelocity + shooterChange);
        Logger.recordOutput("Shooter/VelocityChange", shooterChange);
        currentSubstate = handleShooterTransitions();
        applyStates();
    }

    public void applyStates() {
        switch (currentSubstate) {
            case STOPPED: 
                shooterIO.setVelocityVoltage(0);
                break;
            case ACTIVE, PREPARING:
                shooterIO.setVelocityVoltage(shooterVelocity + shooterChange);
                break;
            case CLEANING:
                shooterIO.setVelocityVoltage(shooterVelocity*0.2);
                break;
        }
    }

    public void setCalculatedShooterVoltage(double distance)
    {
        if(currentSubstate == Substate.ACTIVE || currentSubstate == Substate.PREPARING)
        {
            //Regression is commented out for now
            // shooterVelocity = ShooterConstants.shooterSpeedMapScoring.get(distance);
        }

    }

    public boolean motorsReady()
    {
        return (leftShooterReady() ? 1:0) + (rightShooterReady() ? 1:0) >= shooterCheck.get();
    }

    public boolean leftShooterReady() {
        return Math.abs(inputs.leftVelocityRotPerSec - (shooterVelocity + shooterChange)) < shootingEpsilon.get();
    }

    public boolean rightShooterReady() {
        return Math.abs(inputs.rightVelocityRotPerSec - (shooterVelocity + shooterChange)) < shootingEpsilon.get();
    }

    public void adjustShooterChangeVelocity(double amount) {
        shooterChange += amount;
    }

    public void setShooterChangeVelocity(double amount){
        shooterChange = amount;
    }

    public void setFerry(boolean isFerry) {
        this.isFerry = isFerry;
    }
}
