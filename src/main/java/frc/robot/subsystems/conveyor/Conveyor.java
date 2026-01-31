package frc.robot.subsystems.conveyor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.conveyor.ConveyorIO;

public class Conveyor extends SubsystemBase{

    public enum Substate {
        STOPPED,
        IDLE,
        REVVING,
        READY
    }
    
    // declare IO & logs
    private final ConveyorIO conveyorIO;
    private double conveyorSpeed;

    public Conveyor(ConveyorIO conveyorIO) 
    {
        this.conveyorIO = conveyorIO;
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
    }

}
