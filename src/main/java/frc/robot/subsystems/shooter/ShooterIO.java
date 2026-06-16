package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;


public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public boolean leftConnected = false;
        public double leftVelocityRotPerSec = 0.0;
        public double leftAppliedVolts = 0.0;
        public double leftCurrentAmps = 0.0;

        public boolean rightConnected = false;
        public double rightVelocityRotPerSec = 0.0;
        public double rightAppliedVolts = 0.0;
        public double rightCurrentAmps = 0.0;
    }

    public default void updateInputs(ShooterIOInputs inputs){}


    public default void setVoltage(double voltage){}

    public default void setVelocityVoltage(double velocity){}
}
