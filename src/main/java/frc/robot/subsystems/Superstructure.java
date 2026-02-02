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

// import frc.robot.subsystems.shooter.ShooterSubsystem;
// import frc.robot.subsystems.swerve.SwerveSubsystem;
// import frc.robot.subsystems.turret.TurretSubsystem;

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
    DRIVING,
    SHOOTING,
    SHOOTINGPREPARE,

  }

  private static @Getter @Setter SuperState desiredSuperState = SuperState.DRIVING;
  private static @Getter @Setter SuperState currentSuperState = SuperState.DRIVING;
  private static SuperState previousSuperState = SuperState.DRIVING;

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

    if (currentSuperState == SuperState.STOPPED)
      handleStopped();
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
    var ready = ready(desiredSuperState);

    return currentSuperState;
  }

  private void applyStates() {
    switch (currentSuperState) {
      case STOPPED:
        drive.stop();
    }
  }

  private void handleStopped() {
    drive.stop();
  }

  // TODO update with 2026 state checker
  /** Transition check */
  private boolean ready(SuperState state) {
    return switch (state) {
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
