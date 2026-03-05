package frc.robot.autos;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;

import edu.wpi.first.wpilibj.XboxController;

/** A factory for creating autonomous programs for a given {@link Auto} */
@SuppressWarnings({ "UnusedMethod", "UnusedVariable", "EmptyBlockTag" })
class AutoFactory {
  private final DriverStation.Alliance alliance;

  private final RobotContainer robotContainer;
  private final Drive drive;
  private final Superstructure superstructure;
  private boolean trajectoriesLoaded = false;

  /**
   * Create a new <code>AutoFactory</code>.
   *
   * @param robotContainer The {@link RobotContainer}
   */
  AutoFactory(
      final DriverStation.Alliance alliance,
      final RobotContainer robotContainer,
      final Drive drive,
      final Superstructure superstructure) {
    this.alliance = alliance;
    this.robotContainer = robotContainer;
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

  Command testPath()
  {
    PathPlannerPath path = loadSegment("TestPath");
    preloadTrajectoryClass(path);
    SequentialCommandGroup c = new SequentialCommandGroup();
    c.addCommands(resetPose(path));
    c.addCommands(follow(path));
    c.addCommands(stationaryAAShoot());
    // c.addCommands(superstructure.AimShooting(new XboxController(-1), () -> FieldConstants.getHubePose().toPose2d()));
    return c;
  }

  Command Depot_S3_TowerLeft(Location Start)
  {
    // Load trajectories
    PathPlannerPath StartToDepot = loadSegment(Start, Location.DEPOT);
    PathPlannerPath DepotToS3 = loadSegment(Location.DEPOT, Location.S3);
    PathPlannerPath S3ToTowerLeft = loadSegment(Location.S3, Location.TOWERLEFT);
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

  Command Outpost_S2_TowerRight(Location Start)
  {
      // Load trajectories
      PathPlannerPath StartToOutpost = loadSegment(Start, Location.OUTPOST);
      PathPlannerPath OutpostToS2 = loadSegment(Location.OUTPOST, Location.S2);
      PathPlannerPath S2ToTowerRight = loadSegment(Location.S2, Location.TOWERRIGHT);
      preloadTrajectoryClass(StartToOutpost);
      preloadTrajectoryClass(OutpostToS2);
      preloadTrajectoryClass(S2ToTowerRight);
  
      SequentialCommandGroup c = new SequentialCommandGroup();
      c.addCommands(resetPose(StartToOutpost));
      c.addCommands(follow(StartToOutpost));
      c.addCommands(Commands.waitSeconds(4));
      c.addCommands(follow(OutpostToS2));
      c.addCommands(stationaryAAShoot());
      c.addCommands(follow(S2ToTowerRight));
      // climb
      return c;
  }

  Command Depot_S3_LStart(Location Start)
  {
      // Load trajectories
      PathPlannerPath StartToDepot = loadSegment(Start, Location.DEPOT);
      PathPlannerPath DepotToS3 = loadSegment(Location.DEPOT, Location.S3);
      PathPlannerPath S3ToLStart = loadSegment(Location.S3, Location.LSTART);
      preloadTrajectoryClass(StartToDepot);
      preloadTrajectoryClass(DepotToS3);
      preloadTrajectoryClass(S3ToLStart);
  
      SequentialCommandGroup c = new SequentialCommandGroup();
      c.addCommands(resetPose(StartToDepot));
      c.addCommands(follow(StartToDepot));
      c.addCommands(Commands.waitSeconds(4));
      c.addCommands(follow(DepotToS3));
      c.addCommands(stationaryAAShoot());
      c.addCommands(follow(S3ToLStart));

      return c;
  }

  Command Outpost_S2_RStart(Location Start)
  {
      // Load trajectories
      PathPlannerPath StartToOutpost = loadSegment(Start, Location.OUTPOST);
      PathPlannerPath OutpostToS2 = loadSegment(Location.OUTPOST, Location.S2);
      PathPlannerPath S2ToRStart = loadSegment(Location.S2, Location.RSTART);
      preloadTrajectoryClass(StartToOutpost);
      preloadTrajectoryClass(OutpostToS2);
      preloadTrajectoryClass(S2ToRStart);
  
      SequentialCommandGroup c = new SequentialCommandGroup();
      c.addCommands(resetPose(StartToOutpost));
      c.addCommands(follow(StartToOutpost));
      c.addCommands(Commands.waitSeconds(4));
      c.addCommands(follow(OutpostToS2));
      c.addCommands(stationaryAAShoot());
      c.addCommands(follow(S2ToRStart));

      return c;
  }

  Command Depot_S3_LTrench(Location Start)
  {
      // Load trajectories
      PathPlannerPath StartToDepot = loadSegment(Start, Location.DEPOT);
      PathPlannerPath DepotToS3 = loadSegment(Location.DEPOT, Location.S3);
      PathPlannerPath S3ToLTrench = loadSegment(Location.S3, Location.LTRENCH);
      preloadTrajectoryClass(StartToDepot);
      preloadTrajectoryClass(DepotToS3);
      preloadTrajectoryClass(S3ToLTrench);
  
      SequentialCommandGroup c = new SequentialCommandGroup();
      c.addCommands(resetPose(StartToDepot));
      c.addCommands(follow(StartToDepot));
      c.addCommands(Commands.waitSeconds(4));
      c.addCommands(follow(DepotToS3));
      c.addCommands(stationaryAAShoot());
      c.addCommands(follow(S3ToLTrench));

      return c;
  }

  Command Outpost_S2_RTrench(Location Start)
  {
      // Load trajectories
      PathPlannerPath StartToOutpost = loadSegment(Start, Location.OUTPOST);
      PathPlannerPath OutpostToS2 = loadSegment(Location.OUTPOST, Location.S2);
      PathPlannerPath S2ToRTrench = loadSegment(Location.S2, Location.RTRENCH);
      preloadTrajectoryClass(StartToOutpost);
      preloadTrajectoryClass(OutpostToS2);
      preloadTrajectoryClass(S2ToRTrench);
  
      SequentialCommandGroup c = new SequentialCommandGroup();
      c.addCommands(resetPose(StartToOutpost));
      c.addCommands(follow(StartToOutpost));
      c.addCommands(Commands.waitSeconds(4));
      c.addCommands(follow(OutpostToS2));
      c.addCommands(stationaryAAShoot());
      c.addCommands(follow(S2ToRTrench));

      return c;
  }

  Command Depot_S3_LTrench_ND_LTrench_S3_LTrench(Location Start)
  {
      // Load trajectories
      PathPlannerPath StartToDepot = loadSegment(Start, Location.DEPOT);
      PathPlannerPath DepotToS3 = loadSegment(Location.DEPOT, Location.S3);
      PathPlannerPath S3ToLTrench = loadSegment(Location.S3, Location.LTRENCH);
      PathPlannerPath LTrenchToND = loadSegment(Location.LTRENCH, Location.ND);
      PathPlannerPath NDToLTrench = loadSegment(Location.ND, Location.LTRENCH);
      PathPlannerPath LTrenchToS3 = loadSegment(Location.LTRENCH, Location.S3);

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

  Command Outpost_S2_RTrench_NF_RTrench_S2_RTrench(Location Start)
  {
      // Load trajectories
      PathPlannerPath StartToOutpost = loadSegment(Start, Location.OUTPOST);
      PathPlannerPath OutpostToS2 = loadSegment(Location.OUTPOST, Location.S2);
      PathPlannerPath S2ToRTrench = loadSegment(Location.S2, Location.RTRENCH);
      PathPlannerPath RTrenchToNF = loadSegment(Location.RTRENCH, Location.NF);
      PathPlannerPath NFToRTrench = loadSegment(Location.NF, Location.RTRENCH);
      PathPlannerPath RTrenchToS2 = loadSegment(Location.RTRENCH, Location.S2);

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

  private Command stationaryAAShoot()
  {
      return superstructure.AutonStationaryAimShooting(() -> FieldConstants.getHubePose().toPose2d()).withTimeout(4);
  }

  private Command alignToPose(Pose2d targetPose)
  {
    return superstructure.AimShooting(new XboxController(0), () ->targetPose).withTimeout(2.0);
  }

  private Command alignToTower()
  {
    return new Command() {};
  }

  private Command intakeWhileFollowing(PathPlannerPath path)
  {
    return follow(path).alongWith(superstructure.setDesiredSuperStateCommand(Superstructure.SuperState.INTAKING));
  }

  // Auto init helpers
  private Command resetPose(PathPlannerPath segment) {
    return runOnce(
        () -> {
          // var correctedTraj =
          // segment.generateTrajectory(new ChassisSpeeds(), new Rotation2d(), null);
          Pose2d pose = AllianceFlipUtil.apply(segment.getStartingHolonomicPose().get());
          // Pose2d pose = segment.getPreviewStartingHolonomicPose();
          // getpreviewstartingholonomicpose didn't work
          // getStartingDifferentialPose worked!!!
          drive.setPose(pose);
        });
  }

  // // Auto init helpers
  // private Command resetPose(final Pose2d pose) {
  //   return runOnce(
  //       () -> {
  //         // Pose2d pose = segment.getPreviewStartingHolonomicPose();
  //         // //getpreviewstartingholonomicpose didn't work
  //         // getStartingDifferentialPose worked!!!

  //         drive.setPose(pose);
  //       });
  // }

  // Path following
  private Command follow(final Location start, final Location end) {
    return follow(loadSegment(start, end));
  }

  // Path following
  private Command follow(PathPlannerPath path) {
    return AutoBuilder.followPath(path);
  }

  

  private void preloadTrajectoryClass(PathPlannerPath firstSegment) {
    // This is done because Java loads classes lazily. Calling this here loads the
    // trajectory class
    // which is used to follow paths and saves user code ms loop time at the start
    // of auto.
    if (!trajectoriesLoaded) {
      trajectoriesLoaded = true;
      var trajectory = new PathPlannerTrajectory(
          firstSegment, drive.getChassisSpeeds(), drive.getPose().getRotation(), null);
    }
  }

  // Load paths
  private PathPlannerPath loadSegment(final Location start, final Location end) {
    var name = "%S_%S".formatted(start, end);
    PathPlannerPath path;

    try {
      path = PathPlannerPath.fromChoreoTrajectory(name);
    } catch (Exception e) {
      e.printStackTrace();
      path = null;
    }

    path.preventFlipping = false;

    // return new AutoSegment(start, end, name, path);
    return path;
  }

  private PathPlannerPath loadSegment(String pathName)
  {
    PathPlannerPath path;
    try {
      path = PathPlannerPath.fromChoreoTrajectory(pathName);
    } catch (Exception e)
    {
      e.printStackTrace();
      path = null;
    }
    path.preventFlipping = false;
    return path;
  }
}
