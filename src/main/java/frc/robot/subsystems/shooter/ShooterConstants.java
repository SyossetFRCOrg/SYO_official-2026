package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterConstants {

    //TODO change these
    public static final int leftMotorID = 17;
    public static final int rightMotorID = 15;

    public static final double left_kP = 0;
    public static final double left_kI = 0.02;
    public static final double left_kD = 0;
    public static final double left_kS = 0.28;
    public static final double left_kV = 0.12;
    public static final double left_kA = 0;

    public static final double right_kP = 0;
    public static final double right_kI = 0.02;
    public static final double right_kD = 0;
    public static final double right_kS = 0.28;
    public static final double right_kV = 0.12;
    public static final double right_kA = 0;



    public static final double leftMaxAcceleration = 0.1;
    public static final double leftMaxJerk = 0.1;

    public static final double rightMaxAcceleration = 0.1;
    public static final double rightMaxJerk = 0.1;

    public static final CANBus canbus = new CANBus("SuperStructure");

    // (distance, desired vel)
    public static InterpolatingDoubleTreeMap shooterSpeedMapScoring =
        new InterpolatingDoubleTreeMap();
    public static InterpolatingDoubleTreeMap timeOfFlightMapScoring =
        new InterpolatingDoubleTreeMap();
    public static InterpolatingDoubleTreeMap shooterSpeedMapFerrying =
        new InterpolatingDoubleTreeMap();

    //TODO Retune with new values in cafeteria
    static {
        shooterSpeedMapScoring.put(3.18, 72.0);
        shooterSpeedMapScoring.put(3.73, 79.0);
        shooterSpeedMapScoring.put(3.7, 78.0);
        shooterSpeedMapScoring.put(2.83, 71.0);
        shooterSpeedMapScoring.put(3.5, 74.0);
        shooterSpeedMapScoring.put(2.67, 70.0);
        shooterSpeedMapScoring.put(4.0, 80.0);
        shooterSpeedMapScoring.put(3.17, 72.0);
        shooterSpeedMapScoring.put(2.45, 66.0);
        shooterSpeedMapScoring.put(3.3, 72.0);
        shooterSpeedMapScoring.put(2.02, 67.0);
        
    }

    public static final double ferrySpeed = 80.0;
}