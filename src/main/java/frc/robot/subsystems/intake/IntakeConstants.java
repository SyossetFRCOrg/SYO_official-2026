package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;

public class IntakeConstants {
    public static final int rollerMotorID = 14;
    public static final int leftArmMotorID = 20;
    public static final int rightArmMotorID = 19;

    public static final double intakePosition = 1.3; 
    public static final double upPosition = 0.0; 

    public static final double roller_kP = 100;
    public static final double roller_kD = 0;
    public static final double roller_kS = 0;
    public static final double roller_kV = 12.0 / 5600.0;
    public static final double roller_kA = 0;


    public static final double arm_kP = 1.5;
    public static final double arm_kD = 0.0;
    public static final double arm_kS = 0.0;
    public static final double arm_kV = 0.0;
    public static final double arm_kA = 0.0;
    public static final double arm_kG =  0.3;

    public static final double rollerMaxAcceleration = 0.1;
    public static final double rollerMaxJerk = 0.1;


    public static final double armMotionMagicCruiseVelocity = 80;
    public static final double armMaxAcceleration = 80;
    public static final double armMaxJerk = 1600;


    public static final CANBus canbus = new CANBus("SuperStructure");
}
