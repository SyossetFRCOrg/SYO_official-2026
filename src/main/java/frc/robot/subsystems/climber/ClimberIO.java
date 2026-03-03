package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

// From last year
public interface ClimberIO {
    
    @AutoLog
    public static class ClimberIOInputs {
        public boolean motorConnected = false;
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double torqueCurrentAmps = 0.0;
    }

    public default void updateInputs(ClimberIOInputs inputs) {}

    /** Sets velocity in radians/sec */
    public default void setVoltage(double voltage) {}

    public default void setTargetRotations(double rotations) {}
}
