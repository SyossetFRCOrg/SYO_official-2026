package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;


public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public boolean leftConnected = false;
        public double leftVelocityRadPerSec = 0.0;
        public double leftAppliedVolts = 0.0;
        public double leftCurrentAmps = 0.0;

        public boolean centerConnected = false;
        public double centerVelocityRadPerSec = 0.0;
        public double centerAppliedVolts = 0.0;
        public double centerCurrentAmps = 0.0;

        public boolean rightConnected = false;
        public double rightVelocityRadPerSec = 0.0;
        public double rightAppliedVolts = 0.0;
        public double rightCurrentAmps = 0.0;
    }

    public default void updateInputs(ShooterIOInputs inputs){}


    public default void setVelocity(double velocity){}
}
