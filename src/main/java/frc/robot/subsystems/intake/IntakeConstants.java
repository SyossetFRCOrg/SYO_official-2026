package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;

public class IntakeConstants {
    //TODO  to be tuned
    public static final int rollerMotorID = 14;
    public static final int hopperMotorID = 100;

    public static final double roller_kP = 100;
    public static final double roller_kD = 0;
    public static final double roller_kS = 0;
    public static final double roller_kV = 12.0 / 5600.0;
    public static final double roller_kA = 0;

     public static final double hopper_kP = 100;
    public static final double hopper_kD = 0;
    public static final double hopper_kS = 0;
    public static final double hopper_kV = 12.0 / 5600.0;
    public static final double hopper_kA = 0;

    public static final double rollerMaxAcceleration = 0.1;
    public static final double rollerMaxJerk = 0.1;

    public static final CANBus canbus = new CANBus("*");
}
