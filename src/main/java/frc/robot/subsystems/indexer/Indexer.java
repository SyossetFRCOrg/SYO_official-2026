// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.Setter;

public class Indexer extends SubsystemBase {
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
    
    private double indexerSpeed = 5;

    public Indexer(IndexerIO indexerIO) 
    {
        this.indexerIO = indexerIO;
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
        indexerIO.updateInputs(inputs);
        applyStates();
        handleIndexerTransitions();
    }
    private Substate handleIndexerTransitions() {
        return desiredSubstate;
  }

    //TODO fix the setVelocity for stopped, 0 velocity seems to run the motor. Is possibly a PID issue (?)

    public void applyStates() {
        switch (currentSubstate) {
            case STOPPED:
                indexerIO.setVelocity(0);
                break;
            case INDEXING:
                indexerIO.setVelocity(indexerSpeed);
                break;
            case REVERSING: 
                indexerIO.setVelocity(-indexerSpeed);
                break;

        }
    }
}
