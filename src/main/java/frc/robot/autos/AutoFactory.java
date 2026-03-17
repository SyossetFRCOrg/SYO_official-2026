package frc.robot.autos;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import java.lang.reflect.Field;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.robot.subsystems.drive.Drive;
import edu.wpi.first.wpilibj.XboxController;

/** A factory for creating autonomous programs for a given {@link Auto} */
@SuppressWarnings({ "UnusedMethod", "UnusedVariable", "EmptyBlockTag" })
class AutoFactory {
  // private final DriverStation.Alliance alliance;

  // private final RobotContainer robotContainer;
  private final Drive drive;
  private final Superstructure superstructure;
  private boolean trajectoriesLoaded = false;

  /**
   * Create a new <code>AutoFactory</code>.
   *
   * @param robotContainer The {@link RobotContainer}
   */
  AutoFactory(
      final Drive drive,
      final Superstructure superstructure) {
    // this.alliance = alliance;
    // this.robotContainer = robotContainer;
    this.drive = drive;
    this.superstructure = superstructure;
  }

  /*
   * Autonomous program factories
   *
   * Factory methods should be added here for each autonomous program.
   * The factory methods must:
   * 1. Be package-private (i.e. no access modifier)
   * 2. Accept no parameters
   * 3. Return a link Command
   */

  Command createIdleCommand() {
    // return
    // superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.STOPPED);
    return Commands.none();
  }

  Command testPath() {
    PathPlannerPath path = loadSegment("TestPath");
    preloadTrajectoryClass(path);
    SequentialCommandGroup c = new SequentialCommandGroup();
    path.flipPath(); // Test if flipping works
    c.addCommands(resetPose(path));
    c.addCommands(follow(path));
    // c.addCommands(stationaryAAShoot());
    // c.addCommands(superstructure.AimShooting(new XboxController(-1), () ->
    // FieldConstants.getHubePose().toPose2d()));
    return c;
  }

  Command Depot_S3_TowerLeft(Location Start) {
    // Load trajectories
    PathPlannerPath StartToDepot = loadSegment(Start.getAllianceName(), Location.DEPOT.getAllianceName());
    PathPlannerPath DepotToS3 = loadSegment(Location.DEPOT.getAllianceName(), Location.S3.getAllianceName());
    PathPlannerPath S3ToTowerLeft = loadSegment(Location.S3.getAllianceName(), Location.TOWERLEFT.getAllianceName());
    preloadTrajectoryClass(StartToDepot);
    preloadTrajectoryClass(DepotToS3);
    preloadTrajectoryClass(S3ToTowerLeft);
    SequentialCommandGroup c = new SequentialCommandGroup();
    c.addCommands(resetPose(StartToDepot));
    c.addCommands(intakeWhileFollowing(StartToDepot));
    c.addCommands(follow(DepotToS3));
    c.addCommands(stationaryAAShoot());
    c.addCommands(follow(S3ToTowerLeft));
    // climb
    return c;
  }

  Command Outpost_S2_TowerRight(Location Start) {
    // Load trajectories
    SequentialCommandGroup c = new SequentialCommandGroup();
    PathPlannerPath StartToOutpost = loadSegment(Start.getAllianceName(), Location.OUTPOST.getAllianceName());
    PathPlannerPath OutpostToS2 = loadSegment(Location.OUTPOST.getAllianceName(), Location.S2.getAllianceName());
    PathPlannerPath S2ToTowerRight = loadSegment(Location.S2.getAllianceName(), Location.TOWERRIGHT.getAllianceName());
    preloadTrajectoryClass(StartToOutpost);
    preloadTrajectoryClass(OutpostToS2);
    preloadTrajectoryClass(S2ToTowerRight);

    c.addCommands(resetPose(StartToOutpost));
    c.addCommands(follow(StartToOutpost));
    c.addCommands(Commands.waitSeconds(4));
    c.addCommands(follow(OutpostToS2));
    c.addCommands(stationaryAAShoot());
    c.addCommands(follow(S2ToTowerRight));
    // climb
    return c;
  }

  Command Depot_S3_LStart(Location Start) {
    // Load trajectories
    PathPlannerPath StartToDepot = loadSegment(Start.getAllianceName(), Location.DEPOT.getAllianceName());
    PathPlannerPath DepotToS3 = loadSegment(Location.DEPOT.getAllianceName(), Location.S3.getAllianceName());
    PathPlannerPath S3ToLStart = loadSegment(Location.S3.getAllianceName(), Location.LSTART.getAllianceName());
    preloadTrajectoryClass(StartToDepot);
    preloadTrajectoryClass(DepotToS3);
    preloadTrajectoryClass(S3ToLStart);

    SequentialCommandGroup c = new SequentialCommandGroup();
    Logger.recordOutput("Segment", "%S_%S".formatted(Start.getAllianceName(), Location.DEPOT.getAllianceName()));
    c.addCommands(resetPose(StartToDepot));
    c.addCommands(putArmDown());
    c.addCommands(intakeWhileFollowing(StartToDepot));
    c.addCommands(Commands.waitSeconds(4));
    c.addCommands(follow(DepotToS3));
    c.addCommands(stationaryAAShoot());
    c.addCommands(follow(S3ToLStart));

    return c;
  }

  Command Outpost_S2_RStart(Location Start) {
    // Load trajectories
    PathPlannerPath StartToOutpost = loadSegment(Start.getAllianceName(), Location.OUTPOST.getAllianceName());
    PathPlannerPath OutpostToS2 = loadSegment(Location.OUTPOST.getAllianceName(), Location.S2.getAllianceName());
    PathPlannerPath S2ToRStart = loadSegment(Location.S2.getAllianceName(), Location.RSTART.getAllianceName());
    preloadTrajectoryClass(StartToOutpost);
    preloadTrajectoryClass(OutpostToS2);
    preloadTrajectoryClass(S2ToRStart);

    SequentialCommandGroup c = new SequentialCommandGroup();
    c.addCommands(resetPose(StartToOutpost));
    c.addCommands(putArmDown());
    c.addCommands(intakeWhileFollowing(StartToOutpost));
    c.addCommands(Commands.waitSeconds(4));
    c.addCommands(follow(OutpostToS2));
    c.addCommands(stationaryAAShoot());
    c.addCommands(follow(S2ToRStart));

    return c;
  }

  Command Depot_S3_LTrench(Location Start) {
    // Load trajectories
    PathPlannerPath StartToDepot = loadSegment(Start.getAllianceName(), Location.DEPOT.getAllianceName());
    PathPlannerPath DepotToS3 = loadSegment(Location.DEPOT.getAllianceName(), Location.S3.getAllianceName());
    PathPlannerPath S3ToLTrench = loadSegment(Location.S3.getAllianceName(), Location.LTRENCH.getAllianceName());
    preloadTrajectoryClass(StartToDepot);
    preloadTrajectoryClass(DepotToS3);
    preloadTrajectoryClass(S3ToLTrench);

    SequentialCommandGroup c = new SequentialCommandGroup();
    c.addCommands(resetPose(StartToDepot));
    c.addCommands(putArmDown());
    c.addCommands(intakeWhileFollowing(StartToDepot));
    c.addCommands(Commands.waitSeconds(4));
    c.addCommands(follow(DepotToS3));
    c.addCommands(stationaryAAShoot());
    c.addCommands(follow(S3ToLTrench));

    return c;
  }

  Command S3_Depot_S3(Location Start) {
    PathPlannerPath StartToS3 = loadSegment(Start.getAllianceName(), Location.S3.getAllianceName());
    PathPlannerPath S3ToDepot = loadSegment(Location.S3.getAllianceName(), Location.DEPOT.getAllianceName());
    PathPlannerPath DepotToS3 = loadSegment(Location.DEPOT.getAllianceName(), Location.S3.getAllianceName());

    preloadTrajectoryClass(StartToS3);
    // preloadTrajectoryClass(S3ToDepot);
    // preloadTrajectoryClass(DepotToS3);

    SequentialCommandGroup c = new SequentialCommandGroup();
    
    c.addCommands(resetPose(StartToS3));
    c.addCommands(follow(StartToS3));
    c.addCommands(alignToPose(FieldConstants.getHubPose().toPose2d()).until(() -> DriveCommands.isAimedAtTarget(drive, () -> drive.getPose().relativeTo(FieldConstants.getHubPose().toPose2d()).getTranslation().getAngle().plus(Rotation2d.k180deg))));
    c.addCommands(shooting(5));
    // c.addCommands(Commands.waitSeconds(2));
    c.addCommands(putArmDown().raceWith(Commands.waitSeconds(1.5)));
    c.addCommands(intakeWhileFollowing(S3ToDepot).raceWith(Commands.waitSeconds(5)));
    // c.addCommands(Commands.waitSeconds(4));
    c.addCommands(follow(DepotToS3));
    c.addCommands(alignToPose(FieldConstants.getHubPose().toPose2d()).until(() -> DriveCommands.isAimedAtTarget(drive, () -> drive.getPose().relativeTo(FieldConstants.getHubPose().toPose2d()).getTranslation().getAngle().plus(Rotation2d.k180deg))));
    c.addCommands(shooting(4));
    // c.addCommands(superstructure.setDesiredSuperStateCommand(SuperState.DRIVING));

    return c;
  }

  Command Outpost_S2_RTrench(Location Start) {
    SequentialCommandGroup c = new SequentialCommandGroup();

    // Load trajectories
    PathPlannerPath StartToOutpost = loadSegment(Start.getAllianceName(), Location.OUTPOST.getAllianceName());
    PathPlannerPath OutpostToS2 = loadSegment(Location.OUTPOST.getAllianceName(), Location.S2.getAllianceName());
    PathPlannerPath S2ToRTrench = loadSegment(Location.S2.getAllianceName(), Location.RTRENCH.getAllianceName());
    preloadTrajectoryClass(StartToOutpost);
    c.addCommands(resetPose(StartToOutpost));
    c.addCommands(follow(StartToOutpost));
    // preloadTrajectoryClass(OutpostToS2);
    c.addCommands(Commands.waitSeconds(4));
    c.addCommands(follow(OutpostToS2));
    // preloadTrajectoryClass(S2ToRTrench);
    c.addCommands(stationaryAAShoot());
    c.addCommands(follow(S2ToRTrench));
    
    

    return c;
  }

  Command Depot_S3_LTrench_LCenter_LTrench_S3_LTrench(Location Start) {
    // Load trajectories
    PathPlannerPath StartToDepot = loadSegment(Start.getAllianceName(), Location.DEPOT.getAllianceName());
    PathPlannerPath DepotToS3 = loadSegment(Location.DEPOT.getAllianceName(), Location.S3.getAllianceName());
    PathPlannerPath S3ToLTrench = loadSegment(Location.S3.getAllianceName(), Location.LTRENCH.getAllianceName());
    PathPlannerPath LTrenchToND = loadSegment(Location.LTRENCH.getAllianceName(), Location.ND.getAllianceName());
    PathPlannerPath NDToLTrench = loadSegment(Location.ND.getAllianceName(), Location.LTRENCH.getAllianceName());
    PathPlannerPath LTrenchToS3 = loadSegment(Location.LTRENCH.getAllianceName(), Location.S3.getAllianceName());

    preloadTrajectoryClass(StartToDepot);
    preloadTrajectoryClass(DepotToS3);
    preloadTrajectoryClass(S3ToLTrench);
    preloadTrajectoryClass(LTrenchToND);
    preloadTrajectoryClass(NDToLTrench);
    preloadTrajectoryClass(LTrenchToS3);

    SequentialCommandGroup c = new SequentialCommandGroup();
    c.addCommands(resetPose(StartToDepot));
    c.addCommands(follow(StartToDepot));
    c.addCommands(Commands.waitSeconds(4));
    c.addCommands(follow(DepotToS3));
    c.addCommands(stationaryAAShoot());
    c.addCommands(follow(S3ToLTrench));
    c.addCommands(follow(LTrenchToND));
    c.addCommands(Commands.waitSeconds(4));
    c.addCommands(follow(NDToLTrench));
    c.addCommands(follow(LTrenchToS3));
    c.addCommands(stationaryAAShoot());
    c.addCommands(follow(S3ToLTrench));

    return c;
  }

  Command Outpost_S2_RTrench_RCenter_RTrench_S2_RTrench(Location Start) {
    // Load trajectories
    PathPlannerPath StartToOutpost = loadSegment(Start.getAllianceName(), Location.OUTPOST.getAllianceName());
    PathPlannerPath OutpostToS2 = loadSegment(Location.OUTPOST.getAllianceName(), Location.S2.getAllianceName());
    PathPlannerPath S2ToRTrench = loadSegment(Location.S2.getAllianceName(), Location.RTRENCH.getAllianceName());
    PathPlannerPath RTrenchToNF = loadSegment(Location.RTRENCH.getAllianceName(), Location.NF.getAllianceName());
    PathPlannerPath NFToRTrench = loadSegment(Location.NF.getAllianceName(), Location.RTRENCH.getAllianceName());
    PathPlannerPath RTrenchToS2 = loadSegment(Location.RTRENCH.getAllianceName(), Location.S2.getAllianceName());

    preloadTrajectoryClass(StartToOutpost);
    preloadTrajectoryClass(OutpostToS2);
    preloadTrajectoryClass(S2ToRTrench);
    preloadTrajectoryClass(RTrenchToNF);
    preloadTrajectoryClass(NFToRTrench);
    preloadTrajectoryClass(RTrenchToS2);

    SequentialCommandGroup c = new SequentialCommandGroup();
    c.addCommands(resetPose(StartToOutpost));
    c.addCommands(follow(StartToOutpost));
    c.addCommands(Commands.waitSeconds(4));
    c.addCommands(follow(OutpostToS2));
    c.addCommands(stationaryAAShoot());
    c.addCommands(follow(S2ToRTrench));
    c.addCommands(follow(RTrenchToNF));
    c.addCommands(Commands.waitSeconds(4));
    c.addCommands(follow(NFToRTrench));
    c.addCommands(follow(RTrenchToS2));
    c.addCommands(stationaryAAShoot());
    c.addCommands(follow(S2ToRTrench));

    return c;
  }

  Command DummyShoot(Location Start) {
    PathPlannerPath StartToDummyShoot;
    if (Start.equals(Location.FLSTART))
      StartToDummyShoot = loadSegment(Start.getAllianceName(), Location.FLDUMMYSHOOT.getAllianceName());
    else if (Start.equals(Location.LSTART))
      StartToDummyShoot = loadSegment(Start.getAllianceName(), Location.LDUMMYSHOOT.getAllianceName());
    else if (Start.equals(Location.RSTART))
      StartToDummyShoot = loadSegment(Start.getAllianceName(), Location.RDUMMYSHOOT.getAllianceName());
    else if (Start.equals(Location.FRSTART))
      StartToDummyShoot = loadSegment(Start.getAllianceName(), Location.FRDUMMYSHOOT.getAllianceName());
    else
      StartToDummyShoot = loadSegment(Start.getAllianceName(), Location.S1.getAllianceName());
    
    preloadTrajectoryClass(StartToDummyShoot);

    SequentialCommandGroup c = new SequentialCommandGroup();
    c.addCommands(resetPose(StartToDummyShoot));
    // c.addCommands(follow(StartToDummyShoot));
    c.addCommands(Commands.waitSeconds(1));
    c.addCommands(stationaryAAShoot());

    return c;
  }

  private Command stationaryAAShoot() {
    return superstructure.AutonStationaryAimShooting(() -> FieldConstants.getHubPose().toPose2d());
  }

  private Command shooting(double timeout){
    return superstructure.setDesiredSuperStateCommand(SuperState.SHOOTING).raceWith(Commands.waitSeconds(timeout));
  }

  @SuppressWarnings("unused")
  private Command alignToPose(Pose2d targetPose) {
    return superstructure.AimShooting(new XboxController(0), () -> targetPose).alongWith(superstructure.MoveArmToPosition(0).raceWith(new WaitCommand(0.6)).andThen(superstructure.MoveArmToPosition(1).raceWith(new WaitCommand(0.6)))).repeatedly();
  }

  @SuppressWarnings("unused")
  private Command alignToTower(Location towerLocation) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    
    double tv = table.getEntry("tv").getDouble(0);
    if (tv == 0) 
      return Commands.none();
    
    double seenId = table.getEntry("tid").getDouble(-1);
    if (!(seenId == 15 && seenId == 16))
      return Commands.none();
    
    switch (towerLocation) {
      //add logic
      default:
        return Commands.none();
    }
  }

  private Command intakeWhileFollowing(PathPlannerPath path) {
    return follow(path).alongWith(superstructure.setDesiredSuperStateCommand(Superstructure.SuperState.INTAKING).alongWith(superstructure.MoveArmToPosition(1.4)));
  }

  private Command putArmDown() {
    return superstructure.MoveArmToPosition(1.4);
  }

  // Auto init helpers
  private Command resetPose(PathPlannerPath segment) {
    return runOnce(
        () -> {
          // var correctedTraj =
          // segment.generateTrajectory(new ChassisSpeeds(), new Rotation2d(), null);
          Pose2d pose = segment.getStartingHolonomicPose().get();
          // Pose2d pose = segment.getPreviewStartingHolonomicPose();
          // getpreviewstartingholonomicpose didn't work
          // getStartingDifferentialPose worked!!!
          drive.setPose(pose);
        });
  }

  // // Auto init helpers
  // private Command resetPose(final Pose2d pose) {
  // return runOnce(
  // () -> {
  // // Pose2d pose = segment.getPreviewStartingHolonomicPose();
  // // //getpreviewstartingholonomicpose didn't work
  // // getStartingDifferentialPose worked!!!

  // drive.setPose(pose);
  // });
  // }

  // Path following
  @SuppressWarnings("unused")
  private Command follow(final Location start, final Location end) {
    return follow(loadSegment(start.getAllianceName(), end.getAllianceName()));
  }

  // Path following
  private Command follow(PathPlannerPath path) {
    path.preventFlipping = true;
    return AutoBuilder.followPath(path);
  }

  private void preloadTrajectoryClass(PathPlannerPath firstSegment) {
    // This is done because Java loads classes lazily. Calling this here loads the
    // trajectory class
    // which is used to follow paths and saves user code ms loop time at the start
    // of auto.
    firstSegment.preventFlipping = true;

    if (!trajectoriesLoaded) {
      trajectoriesLoaded = true;
      @SuppressWarnings("unused")
      var trajectory = new PathPlannerTrajectory(
          firstSegment, drive.getChassisSpeeds(), drive.getPose().getRotation(), Drive.PP_CONFIG);
    }
  }

  // Load paths
  private PathPlannerPath loadSegment(final String start, final String end) {
    var name = "%S_%S".formatted(start, end);
    PathPlannerPath path;

  
    try {
      path = PathPlannerPath.fromPathFile(name);
    } catch (Exception e) {
      e.printStackTrace();
      path = null;
    }

    path.preventFlipping = true;

    // return new AutoSegment(start, end, name, path);
    return path;
  }

  private PathPlannerPath loadSegment(String pathName) {
    PathPlannerPath path;
    try {
      path = PathPlannerPath.fromPathFile(pathName);
    } catch (Exception e) {
      e.printStackTrace();
      path = null;
    }
    path.preventFlipping = true;
    return path;
  }
}
