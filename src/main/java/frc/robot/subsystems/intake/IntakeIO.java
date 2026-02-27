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

        public boolean hopperConnected = false;
        public double hopperPosition = 0.0;
        public double hopperVelocityRadPerSec = 0.0;
        public double hopperAppliedVolts = 0.0;

    }

    public default void updateInputs(IntakeIOInputs inputs) {}
    
    // velocity should be in rad/sec
    public default void setRollerVelocity(double velocity) {}

    public default void setHopperEncoderPosition(double positionRadians){}

    public default void moveHopperToPosition(double positionRadians){}
}
