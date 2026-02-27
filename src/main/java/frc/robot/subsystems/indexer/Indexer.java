// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ToggleableSubsystem;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.Setter;

public class Indexer extends ToggleableSubsystem {
  public enum Substate {
        STOPPED,
        INDEXING,
        REVERSING
    }


    // declare IO & logs
    private final IndexerIO indexerIO;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
    private @Getter Substate currentSubstate = Substate.STOPPED;
    private @Setter Substate desiredSubstate = Substate.STOPPED;
    
    private LoggedTunableNumber indexerSpeed = new LoggedTunableNumber("Indexer/IndexerSpeed", 5);

    public Indexer(IndexerIO indexerIO) 
    {
        this.indexerIO = indexerIO;
    }

    @Override
    public void periodic() {
        super.periodic();
        indexerIO.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);
        Logger.recordOutput("Indexer/CurrentSubstate", currentSubstate.toString());
        Logger.recordOutput("Indexer/DesiredSubstate", desiredSubstate.toString());
        
        currentSubstate = handleIndexerTransitions();
        applyStates();
    }
    private Substate handleIndexerTransitions() {
        return desiredSubstate;
  }

    public void applyStates() {
        switch (currentSubstate) {
            case STOPPED:
                indexerIO.setVelocity(0);
                break;
            case INDEXING:
                indexerIO.setVelocity(indexerSpeed.get());
                break;
            case REVERSING: 
                indexerIO.setVelocity(-indexerSpeed.get());
                break;

        }
    }
}
