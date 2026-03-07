package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  private @Getter Drive drive;
  private @Getter RobotContainer container;
  private @Getter Indexer indexer;
  private @Getter Intake intake;
  private @Getter Shooter shooter;
  // private @Getter Climber climber;

  public static enum SuperState {
    // MANUAL,
    STOPPED,
    CLIMBUP,
    CLIMBDOWN,
    INTAKING,
    DRIVING,
    SHOOTING,
    SHOOTINGPREPARE,
    SHOOTINGWHILEINDEXEROUT,
    INTAKINGANDINDEXINGWITHOUTSHOOTING,
    AUTOALIGNING
  }

  private static @Getter @Setter SuperState desiredSuperState = SuperState.DRIVING;
  private static @Getter @Setter SuperState currentSuperState = SuperState.DRIVING;
  private static SuperState previousSuperState = SuperState.DRIVING;

  public Superstructure(RobotContainer container, Drive drive, Indexer indexer, Intake intake,
      Shooter shooter) {
    this.drive = drive;
    this.container = container;
    this.indexer = indexer;
    this.intake = intake;
    this.shooter = shooter;
    // this.climber = climber;
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

    Logger.recordOutput("Drive/EstimatedPose", drive.getPose());
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
    // if (desiredSuperState == SuperState.PREPCLIMBING) {
    //   climber.setDesiredSubstate(climber.getCurrentSubstate() == Climber.Substate.UP ? Climber.Substate.DOWN : Climber.Substate.UP);
    // }
    return switch (desiredSuperState) {
      case SHOOTINGPREPARE -> SuperState.SHOOTINGPREPARE;
      case SHOOTINGWHILEINDEXEROUT -> SuperState.SHOOTINGWHILEINDEXEROUT;
      case SHOOTING -> ready ? SuperState.SHOOTING : SuperState.SHOOTINGPREPARE;
      case INTAKING -> SuperState.INTAKING;
      case INTAKINGANDINDEXINGWITHOUTSHOOTING -> SuperState.INTAKINGANDINDEXINGWITHOUTSHOOTING;
      case AUTOALIGNING -> SuperState.AUTOALIGNING;
      case CLIMBUP -> SuperState.CLIMBUP;
      case CLIMBDOWN -> SuperState.CLIMBDOWN;
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
      case DRIVING:
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
        // drive.stopWithX();
        indexer.setDesiredSubstate(Indexer.Substate.STOPPED);
        intake.setDesiredSubstate(Intake.Substate.STOPPED);
        shooter.setDesiredSubstate(Shooter.Substate.ACTIVE);
        shooter.setCalculatedShooterVoltage(drive.getPose().getTranslation().getDistance(FieldConstants.getHubPose().getTranslation().toTranslation2d()));
        break;
      case SHOOTINGWHILEINDEXEROUT:
        indexer.setDesiredSubstate(Indexer.Substate.REVERSING);
        shooter.setDesiredSubstate(Shooter.Substate.ACTIVE);
        shooter.setCalculatedShooterVoltage(drive.getPose().getTranslation().getDistance(FieldConstants.getHubPose().getTranslation().toTranslation2d()));
        break;
      case INTAKINGANDINDEXINGWITHOUTSHOOTING:
        indexer.setDesiredSubstate(Indexer.Substate.INDEXING);
        intake.setDesiredSubstate(Intake.Substate.ACTIVE);
        shooter.setDesiredSubstate(Shooter.Substate.STOPPED);
        break;
      case SHOOTING:
        indexer.setDesiredSubstate(Indexer.Substate.INDEXING);
        intake.setDesiredSubstate(Intake.Substate.ACTIVE);
        shooter.setDesiredSubstate(Shooter.Substate.ACTIVE);
        break;
      case AUTOALIGNING:
        indexer.setDesiredSubstate(Indexer.Substate.STOPPED);
        intake.setDesiredSubstate(Intake.Substate.STOPPED);
        shooter.setDesiredSubstate(Shooter.Substate.ACTIVE);
        shooter.setCalculatedShooterVoltage(drive.getPose().getTranslation().getDistance(FieldConstants.getHubPose().getTranslation().toTranslation2d()));
        break;
      case CLIMBUP: // TODO for climbup and climbdown, should we stop everything else? if not, we may just be able to set the climber directly
        indexer.setDesiredSubstate(Indexer.Substate.STOPPED);
        intake.setDesiredSubstate(Intake.Substate.STOPPED);
        shooter.setDesiredSubstate(Shooter.Substate.STOPPED);
        // climber.setDesiredSubstate(Climber.Substate.UP);
        break;
      case CLIMBDOWN:
        indexer.setDesiredSubstate(Indexer.Substate.STOPPED);
        intake.setDesiredSubstate(Intake.Substate.STOPPED);
        shooter.setDesiredSubstate(Shooter.Substate.STOPPED);
        // climber.setDesiredSubstate(Climber.Substate.DOWN);
        break;
      default: break;
    }
  }

  // TODO update with 2026 state checker
  /** Transition check */
  private boolean ready(SuperState state) {
    return switch (state) {
      case SHOOTING -> shooter.getCurrentSubstate() == Shooter.Substate.ACTIVE;
      case  INTAKING -> intake.getCurrentSubstate() == Intake.Substate.ACTIVE;
      case STOPPED, DRIVING, CLIMBUP, CLIMBDOWN -> true;
      default -> false;
    };
  }

  public Command AutonStationaryAimShooting(Supplier<Pose2d> targetPose)
  {
    return DriveCommands.joystickDriveFacingPose(
        drive, () -> 0.0, () -> 0.0, targetPose)
        .alongWith(setDesiredSuperStateCommand(SuperState.SHOOTINGPREPARE))
        .andThen(Commands.waitSeconds(3), setDesiredSuperStateCommand(SuperState.DRIVING));
  }

  // flip x and y cuz it works. bad fix
  public Command AimShooting(XboxController controller, Supplier<Pose2d> targetPose) {
    return DriveCommands.joystickDriveFacingPose(
        drive, () -> 0.5 * -controller.getLeftY(), () -> 0.5 * -controller.getLeftX(), targetPose)
        .alongWith(new InstantCommand(() -> {
          RobotState.getInstance().setAutoAiming(true);
          Logger.recordOutput("RobotState/isAutoAiming", RobotState.getInstance().isAutoAiming());
        
        }));
  }
  // public Command SetHopperVoltage(double voltage){
  //   return new InstantCommand(() -> intake.setHopperVoltage(voltage));
  // }
  public Command SetArmVoltage(double voltage){
    return new InstantCommand(() -> intake.setArmVoltage(voltage));
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
