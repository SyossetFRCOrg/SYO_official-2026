package frc.robot.subsystems.indexer;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    
    // required fields
    @AutoLog
    public static class ConveyorIOInputs 
    {
        public boolean connected = false;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    public default void updateInputs(ConveyorIOInputs inputs) {}
  
    // velocity should be in rad/sec
    public default void setVelocity(double velocity) {}   
    
}
