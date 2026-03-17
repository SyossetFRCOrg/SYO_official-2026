package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterConstants {

    public static final int leftMotorID = 17;
    public static final int centerMotorID = 16;
    public static final int rightMotorID = 15;

    public static final double left_kP = 0;
    public static final double left_kI = 0.02;
    public static final double left_kD = 0;
    public static final double left_kS = 0.28;
    public static final double left_kV = 0.12;
    public static final double left_kA = 0;

    public static final double center_kP = 0;
    public static final double center_kI = 0.02;
    public static final double center_kD = 0;
    public static final double center_kS = 0.28;
    public static final double center_kV = 0.12;
    public static final double center_kA = 0;

    public static final double right_kP = 0;
    public static final double right_kI = 0.02;
    public static final double right_kD = 0;
    public static final double right_kS = 0.28;
    public static final double right_kV = 0.12;
    public static final double right_kA = 0;



    public static final double leftMaxAcceleration = 0.1;
    public static final double leftMaxJerk = 0.1;

    public static final double centerMaxAcceleration = 0.1;
    public static final double centerMaxJerk = 0.1;

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
        shooterSpeedMapScoring.put(2.95, 76.0);
        shooterSpeedMapScoring.put(2.11, 67.0);
        shooterSpeedMapScoring.put(3.43, 79.0);
        shooterSpeedMapScoring.put(1.94, 66.0);
        shooterSpeedMapScoring.put(1.70, 65.0);
        shooterSpeedMapScoring.put(4.28, 96.0);
        shooterSpeedMapScoring.put(2.03, 67.0);
        shooterSpeedMapScoring.put(3.17, 82.0);
        shooterSpeedMapScoring.put(3.48, 78.0);
        shooterSpeedMapScoring.put(2.51, 72.0);
        shooterSpeedMapScoring.put(2.9, 70.0);
        shooterSpeedMapScoring.put(2.95, 77.0);
        shooterSpeedMapScoring.put(2.67, 74.0);
        shooterSpeedMapScoring.put(2.6, 74.0);
        shooterSpeedMapScoring.put(2.71, 72.0);
        shooterSpeedMapScoring.put(2.844, 79.7);
        shooterSpeedMapScoring.put(2.74, 78.97);
        shooterSpeedMapScoring.put(2.24, 72.25);
        shooterSpeedMapScoring.put(1.73, 67.18);
        shooterSpeedMapScoring.put(1.46, 65.0);
        
        //TODO: Add ferrying values
        shooterSpeedMapFerrying.put(1.0, 65.0);
    }

    public static final double constantFerrySpeed = 80.0;
}