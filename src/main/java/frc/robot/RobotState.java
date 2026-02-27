package frc.robot;

import edu.wpi.first.math.geometry.*;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.util.GeomUtil;
import frc.robot.util.swerve.ModuleLimits;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.AutoLogOutput;

@ExtensionMethod({ GeomUtil.class })
public class RobotState {

  private RobotState() {
  }

  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null)
      instance = new RobotState();
    return instance;
  }

  @AutoLogOutput(key = "RobotState/RobotPose")
  @Getter
  @Setter
  private volatile Pose2d RobotPose = new Pose2d();

  @AutoLogOutput(key = "RobotState/addingVision")
  @Getter
  @Setter
  private volatile boolean addingVision = true;

  @AutoLogOutput(key = "RobotState/isAutoAiming")
  @Getter
  @Setter
  private volatile boolean isAutoAiming = true;

  @AutoLogOutput(key = "RobotState/tuningTempPose")
  @Getter
  @Setter
  private volatile Pose2d tuningTempPose = null;

  // for practice fields like Arumdaun

  @AutoLogOutput(key = "Swerve/ModuleLimits")
  public ModuleLimits getModuleLimits() {
    // if (DriverStation.isTeleop()) {
    return TunerConstants.moduleLimitsFree;
  }
  
}
