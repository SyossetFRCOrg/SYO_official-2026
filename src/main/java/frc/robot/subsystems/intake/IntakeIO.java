package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

    // required fields
    @AutoLog
    public static class IntakeIOInputs 
    {
        public boolean rollerConnected = false;
        public double rollerVelocityRadPerSec = 0.0;
        public double rollerAppliedVolts = 0.0;

        public boolean armConnected = false;
        public double armPosition = 0.0;
        public double armVelocityRadPerSec = 0.0;
        public double armAppliedVolts = 0.0;

    }

    public default void updateInputs(IntakeIOInputs inputs) {}
    
    // velocity should be in rad/sec
    public default void setRollerVoltage(double velocity) {}

    public default void setArmVoltage(double voltage){}

    public default void moveArmToPosition(double positionRotation){}


}
