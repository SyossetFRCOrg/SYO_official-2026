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
        PREPARING,
        READY
    }

 // declare IO & logs
    private final IndexerIO indexerIO;
    private @Getter Substate currentSubstate = Substate.STOPPED;
    private @Setter Substate desiredSubstate = Substate.STOPPED;
    
    private double indexerSpeed;

    public Indexer(IndexerIO indexerIO) 
    {
        this.indexerIO = indexerIO;
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
    }
}
