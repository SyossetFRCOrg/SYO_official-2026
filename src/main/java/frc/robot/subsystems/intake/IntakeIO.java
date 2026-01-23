package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

    // required fields
    @AutoLog
    public static class IntakeIOInputs 
    {
        public boolean connected = false;
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}
    
    // velocity should be in rad/sec
    public default void setVelocity(double velocity) {}

}
