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
    PathPlannerPath path = loadSegment("BLUE_DS1_BASIC");
    preloadTrajectoryClass(path);
    SequentialCommandGroup c = new SequentialCommandGroup();
    c.addCommands(follow(path));
    c.addCommands(stationaryAAShoot());
    c.addCommands(follow(path));
    return c;
  }

  
  private Command stationaryAAShoot()
  {
      return superstructure.AutonStationaryAimShooting(() -> FieldConstants.getHubePose().toPose2d());
  }

  private Command intakeWhileFollowing(PathPlannerPath path)
  {
    return follow(path).alongWith(superstructure.setDesiredSuperStateCommand(Superstructure.SuperState.INTAKING));
  }

  // Auto init helpers
  private Command resetPose(final PathPlannerPath segment) {
    return runOnce(
        () -> {

          // var correctedTraj =
          // segment.generateTrajectory(new ChassisSpeeds(), new Rotation2d(), null);
          // Pose2d pose = correctedTraj.getInitialPose();
          Pose2d pose = AllianceFlipUtil.apply(segment.getStartingHolonomicPose().get());
          // Pose2d pose = segment.getPreviewStartingHolonomicPose();
          // //getpreviewstartingholonomicpose didn't work
          // getStartingDifferentialPose worked!!!

          drive.setPose(pose);
        });
  }

  // Auto init helpers
  private Command resetPose(final Pose2d pose) {
    return runOnce(
        () -> {
          // Pose2d pose = segment.getPreviewStartingHolonomicPose();
          // //getpreviewstartingholonomicpose didn't work
          // getStartingDifferentialPose worked!!!

          drive.setPose(pose);
        });
  }

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
    var name = "%S_TO_%S".formatted(start, end);
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
