package frc.robot.subsystems.shooter;

import java.util.HashMap;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ToggleableSubsystem;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.Setter;

public class Shooter extends ToggleableSubsystem {
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

    private final Timer debounceTimer = new Timer();
    private final double toleranceTime = 0.1;

    private static final HashMap<Substate, LoggedTunableNumber> initializeSpeeds() {
        HashMap<Substate, LoggedTunableNumber> map = new HashMap<Substate, LoggedTunableNumber>();
        map.put(Substate.ACTIVE, new LoggedTunableNumber("Active Shooter Speed", shooterSpeed));
        //maps
        return map;
    }

    private @Getter Substate currentSubstate = Substate.STOPPED;
    private @Setter Substate desiredSubstate = Substate.STOPPED;

    private static final HashMap<Substate, LoggedTunableNumber> shooterSpeeds = initializeSpeeds();
    private static double shooterSpeed = .5;

    
    private Substate handleShooterTransitions() {
        return desiredSubstate;
    }

    @Override
    public void periodic() {
        super.periodic();
        shooterIO.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        handleShooterTransitions();
        applyStates();
    }

    public void applyStates() {
        switch (currentSubstate) {
            case STOPPED: 
                shooterIO.setVelocity(0);
                break;
            case ACTIVE, PREPARING:
                shooterIO.setVelocity(shooterSpeeds.get(currentSubstate).get());
                break;   
        }
    }

}
