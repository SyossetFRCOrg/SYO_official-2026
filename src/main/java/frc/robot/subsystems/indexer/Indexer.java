package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.indexer.IndexerIO;

public class Indexer extends SubsystemBase {

    // declare IO & logs
    private final IndexerIO indexerIO;
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
