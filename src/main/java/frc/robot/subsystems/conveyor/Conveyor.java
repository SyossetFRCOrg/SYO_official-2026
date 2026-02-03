package frc.robot.subsystems.conveyor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.conveyor.ConveyorIO;
import lombok.Getter;
import lombok.Setter;

public class Conveyor extends SubsystemBase{

    public enum Substate {
        STOPPED,
        PREPARING,
        READY
    }

    // declare IO & logs
    private final ConveyorIO conveyorIO;
    private @Getter Substate currentSubstate = Substate.STOPPED;
    private @Setter Substate desiredSubstate = Substate.STOPPED;


    private double conveyorSpeed;

    public Conveyor(ConveyorIO conveyorIO) 
    {
        this.conveyorIO = conveyorIO;
    }

    @Override
    public void periodic() {
        super.periodic();
    }

}
