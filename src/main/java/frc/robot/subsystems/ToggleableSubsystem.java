package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class ToggleableSubsystem extends SubsystemBase {
    
    private boolean enabled = true;

    @Override
    public void periodic() {
        if (!enabled) return;
        super.periodic();
    }

    public void disable(String logName) {
        enabled = false;
        
        System.out.println(logName + " disabled!");
    }

    // This may not ever be necessary, but just in case
    public void enable(String logName) {
        enabled = true;
        System.out.println(logName + enabled);
        periodic();
    }
}
