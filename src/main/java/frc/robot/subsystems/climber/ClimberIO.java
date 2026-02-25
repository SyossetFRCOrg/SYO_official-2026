package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

// From last year
public interface ClimberIO {
    
    @AutoLog
    public static class ClimberIOInputs {
        public boolean connectedMotor1 = false;
        public double positionRad1 = 0.0;
        public double velocityRadPerSec1 = 0.0;
        public double appliedVolts1 = 0.0;
        public double currentAmps1 = 0.0;

        public boolean connectedMotor2 = false;
        public double positionRad2 = 0.0;
        public double velocityRadPerSec2 = 0.0;
        public double appliedVolts2 = 0.0;
        public double currentAmps2 = 0.0;
    }

    public default void updateInputs(ClimberIOInputs inputs) {}

    /** Sets velocity in radians/sec */
    public default void setVoltage(double voltage) {}
}
