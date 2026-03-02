package frc.robot.subsystems.climber;

import com.ctre.phoenix6.CANBus;

// Delete if not needed
public class ClimberConstants {
    //TODO set to actual ID
    public static final int motor1ID = 21;
    public static final int motor2ID = -1;
    
    //TODO Tune (This is all from indexer)
    public static final double kP = 100;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double kV = 12.0 / 5600.0;
    public static final double kA = 0;

    public static final double maxAcceleration = 0.1;
    public static final double maxJerk = 0.1;

    public static final CANBus canbus = new CANBus("*");
}
