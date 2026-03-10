package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;

public class IntakeConstants {
    public static final int rollerMotorID = 14;
    public static final int armMotorID = 19;

    public static final double intakePosition = 0.0; 
    public static final double upPosition = 1.0; //TODO Find

    public static final double roller_kP = 100;
    public static final double roller_kD = 0;
    public static final double roller_kS = 0;
    public static final double roller_kV = 12.0 / 5600.0;
    public static final double roller_kA = 0;


    public static final double arm_kP = 4.8;
    public static final double arm_kD = 0.1;
    public static final double arm_kS = 0.25;
    public static final double arm_kV = 0.12;
    public static final double arm_kA = 0.01;

    public static final double rollerMaxAcceleration = 0.1;
    public static final double rollerMaxJerk = 0.1;


    public static final double armMotionMagicCruiseVelocity = 70;
    public static final double armMaxAcceleration = 70;
    public static final double armMaxJerk = 1600;


    public static final CANBus canbus = new CANBus("*");
}
