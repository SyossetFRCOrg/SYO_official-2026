package frc.robot;

import edu.wpi.first.math.geometry.*;
import frc.robot.generated.TunerConstants;
// import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import frc.robot.util.GeomUtil;
import frc.robot.util.swerve.ModuleLimits;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.ExtensionMethod;
// import org.littletonrobotics.frc2024.subsystems.drive.DriveConstants;
// import org.littletonrobotics.frc2024.subsystems.superstructure.arm.ArmConstants;
// import org.littletonrobotics.frc2024.util.AllianceFlipUtil;
// import org.littletonrobotics.frc2024.util.GeomUtil;
// import org.littletonrobotics.frc2024.util.LoggedTunableNumber;
// import org.littletonrobotics.frc2024.util.NoteVisualizer;
// import org.littletonrobotics.frc2024.util.swerve.ModuleLimits;
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

  @AutoLogOutput(key = "RobotState/tuningTempPose")
  @Getter
  @Setter
  private volatile Pose2d tuningTempPose = null;

  // in order, blue A-L and then red A-L
  public static final int[] tagList = { 18, 17, 22, 21, 20, 19, 7, 8, 9, 10, 11, 6 };

  // for practice fields like Arumdaun

  @AutoLogOutput(key = "Swerve/ModuleLimits")
  public ModuleLimits getModuleLimits() {
    // if (DriverStation.isTeleop()) {
    return TunerConstants.moduleLimitsFree;
  }
  
}
