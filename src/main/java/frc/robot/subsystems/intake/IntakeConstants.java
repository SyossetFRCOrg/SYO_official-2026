package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;

public class IntakeConstants {
    public static final int rollerMotorID = 14;
    public static final int hopperMotorID = 20;
    public static final int armMotorID = 19;


    //TODO  to be tuned
    public static final double roller_kP = 100;
    public static final double roller_kD = 0;
    public static final double roller_kS = 0;
    public static final double roller_kV = 12.0 / 5600.0;
    public static final double roller_kA = 0;

    public static final double hopper_kP = 4.8;
    public static final double hopper_kD = 0.1;
    public static final double hopper_kS = 0.25;
    public static final double hopper_kV = 0.12;
    public static final double hopper_kA = 0.01;

    public static final double arm_kP = 4.8;
    public static final double arm_kD = 0.1;
    public static final double arm_kS = 0.25;
    public static final double arm_kV = 0.12;
    public static final double arm_kA = 0.01;

    public static final double rollerMaxAcceleration = 0.1;
    public static final double rollerMaxJerk = 0.1;

    public static final double hopperMotionMagicCruiseVelocity = 80;
    public static final double hopperMotionMagicMaxAcceleration = 160;
    public static final double hopperMotionMagicMaxJerk = 1600;

    public static final double armMotionMagicCruiseVelocity = 80;
    public static final double armMaxAcceleration = 160;
    public static final double armMaxJerk = 1600;


    public static final CANBus canbus = new CANBus("*");
}
