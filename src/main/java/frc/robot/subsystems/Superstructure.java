package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
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

  public static enum SuperState {
    // MANUAL,
    STOPPED,
    DRIVING,
    SHOOTING,
    SHOOTINGPREPARE,

  }

  private static @Getter @Setter SuperState desiredState = SuperState.DRIVING;
  private static @Getter @Setter SuperState currentState = SuperState.DRIVING;
  private static SuperState previousState = SuperState.DRIVING;



  public Superstructure(Drive drive, RobotContainer container) {
    this.drive = drive;
    this.container = container;
  }

  @Override
  public void periodic() {
    currentState = handleStateTransitions();    

    if (RobotState.getInstance().getTuningTempPose() != null) {
      Logger.recordOutput(
          "RobotState/tuningTempPose",
          new double[] {
            RobotState.getInstance().getTuningTempPose().getX(),
            RobotState.getInstance().getTuningTempPose().getY(),
            RobotState.getInstance().getTuningTempPose().getRotation().getDegrees()
          });
    } else {
      Logger.recordOutput("RobotState/tuningTempPose", new double[] {0, 0, 0});
    }

    Logger.recordOutput(
        "Drive/EstimatedPose",
        new double[] {
          drive.getPose().getX(), drive.getPose().getY(), drive.getPose().getRotation().getDegrees()
        });

    Logger.recordOutput("Superstructure/CurrentSuperState", currentState.toString());
    Logger.recordOutput("Superstructure/DesiredSuperState", desiredState.toString());

    if (currentState == SuperState.STOPPED) handleStopped();
  }

  /**
   * Sets currentState to the appropiate transition state based on desiredState
   *
   * @return The current super state
   */
  private SuperState handleStateTransitions() {
    previousState = currentState;
    var ready = ready(desiredState);
    // currentState =
    //     switch (desiredState) {
    //       case L1 -> ready ? SuperState.L1 : SuperState.L1PREPARE;
    //       case L2 -> ready ? SuperState.L2 : SuperState.L2PREPARE;
    //       case L3 -> ready ? SuperState.L3 : SuperState.L3PREPARE;
    //       case L4 -> ready ? SuperState.L4 : SuperState.L4PREPARE;
    //       case INTAKE -> ready ? SuperState.INTAKE : SuperState.INTAKEPREPARE;
    //       case INTAKELOW -> ready ? SuperState.INTAKELOW : SuperState.INTAKELOWPREPARE;
    //       default -> desiredState;
    //     };

    return currentState;
  }

  private void handleStopped() {
    drive.stop();
  }

  //TODO update with 2026 state checker
  /** Transition check */
  private boolean ready(SuperState state) {
    return switch (state) {
      default -> false;
    };
  }

  public BooleanSupplier doesCommandMatch(SuperState currentState) {
    return () -> Superstructure.currentState == currentState;
  }

  /** State pushers */
  public void setWantedSuperState(SuperState desiredState) {
    Superstructure.desiredState = desiredState;
  }

  public Command setWantedSuperStateCommand(SuperState desiredState) {
    return new InstantCommand(
        () -> {
          setWantedSuperState(desiredState);
        });
  }
}
