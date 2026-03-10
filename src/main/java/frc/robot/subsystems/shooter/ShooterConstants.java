package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

//TODO adjust these values
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

    public static final CANBus canbus = new CANBus("*");

    // (distance, desired vel)
    public static InterpolatingDoubleTreeMap shooterSpeedMapScoring =
        new InterpolatingDoubleTreeMap();
    public static InterpolatingDoubleTreeMap timeOfFlightMapScoring =
        new InterpolatingDoubleTreeMap();

    public static InterpolatingDoubleTreeMap shooterSpeedMapFerrying =
        new InterpolatingDoubleTreeMap();

    //TODO Interpolation
    static {
        shooterSpeedMapScoring.put(0.0,0.0);
        shooterSpeedMapScoring.put(2.95, 76.0);
        shooterSpeedMapScoring.put(2.11,67.0);
        shooterSpeedMapScoring.put(3.43,79.0);
        shooterSpeedMapScoring.put(1.94,66.0);
        shooterSpeedMapScoring.put(1.7,65.0);
        shooterSpeedMapScoring.put(4.28,96.0);
        shooterSpeedMapScoring.put(2.03,67.0);

    }
}