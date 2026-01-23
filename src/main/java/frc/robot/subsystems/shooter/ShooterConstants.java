package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;

//TODO adjust these values
public class ShooterConstants {

    public static final int motorID = 1;

    public static final double kP = 100;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double kV = 12.0 / 5600.0;
    public static final double kA = 0;

    public static final double maxAcceleration = 0.1;
    public static final double maxJerk = 0.1;

    public static final CANBus canBus = new CANBus();

}