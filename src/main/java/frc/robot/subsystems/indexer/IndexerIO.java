package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {

    @AutoLog
    public static class IndexerIOInputs
    {
        public boolean connected = false;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmpts = 0.0;
    }

    public default void updateInputs(IndexerIOInputs inputs) {}

    public default void setVelocity(double velocity) {}
}
