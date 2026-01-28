package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    public static enum SubState {
        IDLE,
        INTAKING,
        
    }
    // declare IO & logs
    private final IntakeIO intakeIO;
    private double intakeSpeed;

    public Intake(IntakeIO intakeIO) 
    {
        this.intakeIO = intakeIO;
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
    }
    
}
