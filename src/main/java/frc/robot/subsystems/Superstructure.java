package frc.robot.subsystems;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
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

  public Superstructure(RobotContainer container, Drive drive, Indexer indexer, Intake intake,
      Shooter shooter) {
    this.drive = drive;
    this.container = container;
    this.indexer = indexer;
    this.intake = intake;
    this.shooter = shooter;
  }

  @Override
  public void periodic() {
    previousSuperState = currentSuperState;
    currentSuperState = handleStateTransitions();
    logRoboStateValues();
    applyStates();
    if (previousSuperState != currentSuperState) {
      System.out.println("Superstructure State Changed from " + previousSuperState + "to " + currentSuperState);
    }
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
    Logger.recordOutput("Superstructure/CurrentSuperState", currentSuperState.toString());
    Logger.recordOutput("Superstructure/DesiredSuperState", desiredSuperState.toString());


    

  }

  /**
   * Sets currentState to the appropiate transition state based on desiredState
   *
   * @return The current super state
   */
  private SuperState handleStateTransitions() {
    previousSuperState = currentSuperState;
    boolean ready = ready(desiredSuperState);
    return switch (desiredSuperState) {
      case SHOOTING -> ready ? SuperState.SHOOTING : SuperState.SHOOTINGPREPARE;
      case INTAKING -> SuperState.INTAKING;
      default -> ready ? desiredSuperState : currentSuperState;
    };
  }

  /**
   * Sets each subsystem to desired substate based on current SuperState
   */
  private void applyStates() {
    switch (currentSuperState) {
      case STOPPED:
        drive.stop();
        indexer.setDesiredSubstate(Indexer.Substate.STOPPED);
        intake.setDesiredSubstate(Intake.Substate.STOPPED);
        shooter.setDesiredSubstate(Shooter.Substate.STOPPED);
        break;
      case IDLE:
        indexer.setDesiredSubstate(Indexer.Substate.STOPPED);
        intake.setDesiredSubstate(Intake.Substate.STOPPED);
        shooter.setDesiredSubstate(Shooter.Substate.STOPPED);
        break;
      case INTAKING:
        indexer.setDesiredSubstate(Indexer.Substate.REVERSING);
        intake.setDesiredSubstate(Intake.Substate.ACTIVE);
        shooter.setDesiredSubstate(Shooter.Substate.STOPPED);
        break;
      case SHOOTINGPREPARE:
        drive.stopWithX();
        intake.setDesiredSubstate(Intake.Substate.STOPPED);
        indexer.setDesiredSubstate(Indexer.Substate.STOPPED);
        shooter.setDesiredSubstate(Shooter.Substate.ACTIVE);
        break;
      case SHOOTING:
        indexer.setDesiredSubstate(Indexer.Substate.INDEXING);
        break;
    }
  }

  // TODO update with 2026 state checker
  /** Transition check */
  private boolean ready(SuperState state) {
    return switch (state) {
      case SHOOTING -> shooter.getCurrentSubstate() == Shooter.Substate.ACTIVE;
      case INTAKING -> intake.getCurrentSubstate() == Intake.Substate.ACTIVE;
      case STOPPED, IDLE -> true;
      default -> false;
    };
  }

  public BooleanSupplier doesCommandMatch(SuperState currentState) {
    return () -> Superstructure.currentSuperState == currentState;
  }

  public Command setDesiredSuperStateCommand(SuperState desiredState) {
    return new InstantCommand(
        () -> {
          setDesiredSuperState(desiredState);
        });
  }
}
