package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;

//TODO adjust these values
public class ShooterConstants {

    public static final int leftMotorID = 17;
    public static final int centerMotorID = 16;
    public static final int rightMotorID = 15;

    public static final double left_kP = 0;
    public static final double left_kD = 0;
    public static final double left_kS = 0;
    public static final double left_kV = 0;
    public static final double left_kA = 0;

    public static final double center_kP = 0;
    public static final double center_kD = 0;
    public static final double center_kS = 0;
    public static final double center_kV = 0;
    public static final double center_kA = 0;

    public static final double right_kP = 0;
    public static final double right_kD = 0;
    public static final double right_kS = 0;
    public static final double right_kV = 0;
    public static final double right_kA = 0;



    public static final double leftMaxAcceleration = 0.1;
    public static final double leftMaxJerk = 0.1;

    public static final double centerMaxAcceleration = 0.1;
    public static final double centerMaxJerk = 0.1;

    public static final double rightMaxAcceleration = 0.1;
    public static final double rightMaxJerk = 0.1;

    public static final CANBus canbus = new CANBus("*");

}