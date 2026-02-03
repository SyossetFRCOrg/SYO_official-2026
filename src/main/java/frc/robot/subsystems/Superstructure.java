package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

import java.util.function.BooleanSupplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;


public class Superstructure extends SubsystemBase {
  private Drive drive;
  private RobotContainer container;
  private Conveyor conveyor;
  private Indexer indexer;
  private Intake intake;
  private Shooter shooter;

  public static enum SuperState {
    // MANUAL,
    STOPPED,
    INTAKING,
    IDLE,
    SHOOTING,
    SHOOTINGPREPARE,

  }

  private static @Getter @Setter SuperState desiredSuperState = SuperState.IDLE;
  private static @Getter @Setter SuperState currentSuperState = SuperState.IDLE;
  private static SuperState previousSuperState = SuperState.IDLE;

  public Superstructure(RobotContainer container, Conveyor conveyor, Drive drive, Indexer indexer, Intake intake,
      Shooter shooter) {
    this.drive = drive;
    this.container = container;
    this.conveyor = conveyor;
    this.indexer = indexer;
    this.intake = intake;
    this.shooter = shooter;
  }

  @Override
  public void periodic() {
    currentSuperState = handleStateTransitions();
    logRoboStateValues();
    applyStates();

  }

  public void logRoboStateValues() {
    if (RobotState.getInstance().getTuningTempPose() != null) {
      Logger.recordOutput(
          "RobotState/tuningTempPose",
          new double[] {
              RobotState.getInstance().getTuningTempPose().getX(),
              RobotState.getInstance().getTuningTempPose().getY(),
              RobotState.getInstance().getTuningTempPose().getRotation().getDegrees()
          });
    } else {
      Logger.recordOutput("RobotState/tuningTempPose", new double[] { 0, 0, 0 });
    }

    Logger.recordOutput(
        "Drive/EstimatedPose",
        new double[] {
            drive.getPose().getX(), drive.getPose().getY(), drive.getPose().getRotation().getDegrees()
        });
    if(currentSuperState != desiredSuperState)
    {
      Logger.recordOutput("Superstructure/CurrentSuperState", currentSuperState.toString());
      Logger.recordOutput("Superstructure/DesiredSuperState", desiredSuperState.toString());
    }

    

  }

  /**
   * Sets currentState to the appropiate transition state based on desiredState
   *
   * @return The current super state
   */
  private SuperState handleStateTransitions() {
    previousSuperState = currentSuperState;
    boolean ready = ready(desiredSuperState);
    currentSuperState = 
      switch(desiredSuperState)
      {
        case SHOOTING -> ready ? SuperState.SHOOTING : SuperState.SHOOTINGPREPARE;
        default -> ready ? desiredSuperState : currentSuperState;
      };
    return currentSuperState;
  }

  private void applyStates() {
    switch (currentSuperState) {
      case STOPPED:
        drive.stop();
        conveyor.setDesiredSubstate(Conveyor.Substate.STOPPED);
        indexer.setDesiredSubstate(Indexer.Substate.STOPPED);
        intake.setDesiredSubstate(Intake.Substate.STOPPED);
        shooter.setDesiredSubstate(Shooter.Substate.STOPPED);
        break;
      case IDLE:
        conveyor.setDesiredSubstate(Conveyor.Substate.READY);
        indexer.setDesiredSubstate(Indexer.Substate.READY);
        intake.setDesiredSubstate(Intake.Substate.READY);
        shooter.setDesiredSubstate(Shooter.Substate.READY);
        break;
      case INTAKING:
        conveyor.setDesiredSubstate(Conveyor.Substate.READY);
        indexer.setDesiredSubstate(Indexer.Substate.STOPPED);
        intake.setDesiredSubstate(Intake.Substate.READY);
        shooter.setDesiredSubstate(Shooter.Substate.READY);
        break;
      case SHOOTINGPREPARE:
        drive.stopWithX();
        intake.setDesiredSubstate(Intake.Substate.STOPPED);
        indexer.setDesiredSubstate(Indexer.Substate.STOPPED);
        shooter.setDesiredSubstate(Shooter.Substate.READY);
        break;
      case SHOOTING:
        drive.stopWithX();
        intake.setDesiredSubstate(Intake.Substate.STOPPED);
        indexer.setDesiredSubstate(Indexer.Substate.READY);
        shooter.setDesiredSubstate(Shooter.Substate.READY);
        break;
    }
  }

  private void handleStopped() {
    drive.stop();
  }

  // TODO update with 2026 state checker
  /** Transition check */
  private boolean ready(SuperState state) {
    return switch (state) {
      case SHOOTING, SHOOTINGPREPARE -> shooter.getCurrentSubstate() == Shooter.Substate.READY &&
                                        indexer.getCurrentSubstate() == Indexer.Substate.READY;
      case INTAKING -> intake.getCurrentSubstate() == Intake.Substate.READY;
      default -> false;
    };
  }

  public BooleanSupplier doesCommandMatch(SuperState currentState) {
    return () -> Superstructure.currentSuperState == currentState;
  }

  /** State pushers */
  public void setWantedSuperState(SuperState desiredState) {
    Superstructure.desiredSuperState = desiredState;
  }

  public Command setWantedSuperStateCommand(SuperState desiredState) {
    return new InstantCommand(
        () -> {
          setWantedSuperState(desiredState);
        });
  }
}
