package frc.robot.subsystems.shooter;

import java.util.HashMap;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.Setter;

public class Shooter extends SubsystemBase{
    public static enum SubState {
        IDLE;
    }
    private static final HashMap<SuperState, LoggedTunableNumber> shooterSpeeds = initializeSpeeds();

    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private final Timer debounceTimer = new Timer();
    private final double toleranceTime = 0.1;

    private static final HashMap<SuperState, LoggedTunableNumber> initializeSpeeds() {
        var map = new HashMap<SuperState, LoggedTunableNumber>();
        return map;
    }
        

    private @Getter @Setter SubState state = SubState.IDLE;

    private double shooterSpeed;
    private final ShooterIO shooterIO;

    public Shooter(ShooterIO shooterIO) {
        this.shooterIO = shooterIO;
    }

    @Override
    public void periodic() {
    shooterIO.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    if (shooterSpeeds.containsKey(Superstructure.getCurrentState())) {
      shooterSpeed = shooterSpeeds.get(Superstructure.getCurrentState()).get();
    }   

    shooterIO.setVelocity(shooterSpeed);

    if (inputs.currentAmps < 1.5) {
      debounceTimer.reset();
    }
    
  }


}   
