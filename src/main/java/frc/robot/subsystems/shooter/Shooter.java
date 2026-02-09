package frc.robot.subsystems.shooter;

import java.util.HashMap;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.Setter;

public class Shooter extends SubsystemBase {

    public enum Substate {
        STOPPED,
        PREPARING,
        ACTIVE
    }

    private static final HashMap<SuperState, LoggedTunableNumber> shooterSpeeds = initializeSpeeds();

    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private final Timer debounceTimer = new Timer();
    private final double toleranceTime = 0.1;

    private static final HashMap<SuperState, LoggedTunableNumber> initializeSpeeds() {
        var map = new HashMap<SuperState, LoggedTunableNumber>();
        return map;
    }

    private @Getter Substate currentSubstate = Substate.STOPPED;
    private @Setter Substate desiredSubstate = Substate.STOPPED;

    private double shooterSpeed = .5;
    private final ShooterIO shooterIO;

    public Shooter(ShooterIO shooterIO) {
        this.shooterIO = shooterIO;
    }
    private Substate handleShooterTransitions() {
        return desiredSubstate;
  }

    @Override
    public void periodic() {
        shooterIO.updateInputs(inputs);
        handleShooterTransitions();
        Logger.processInputs("Shooter", inputs);
        applyStates();
    }

    public void applyStates() {
        switch (currentSubstate) {
            case STOPPED: shooterIO.setVelocity(0);
            case ACTIVE, PREPARING: shooterIO.setVelocity(shooterSpeed);
        }
    }

}
