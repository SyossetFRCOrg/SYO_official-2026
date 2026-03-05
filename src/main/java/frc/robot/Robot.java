package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.autos.AutoChooser;
import frc.robot.subsystems.drive.Drive;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command autonomousCommand;

  private final Field2d field = new Field2d();

  private AutoChooser autoChooser;

  private final RobotContainer robotContainer;

  public Robot() {
    robotContainer = new RobotContainer();
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    Logger.start();

    Drive drive = robotContainer.getDrive();

    SmartDashboard.putData("Swerve Drive", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");

        builder.addDoubleProperty("Front Left Angle", () -> drive.getModules()[0].getAngle().getRadians() + (DriverStation.getAlliance().get().equals(Alliance.Red) ? Math.PI : 0), null);
        builder.addDoubleProperty("Front Left Velocity", () -> drive.getModules()[0].getVelocityMetersPerSec(), null);

        builder.addDoubleProperty("Front Right Angle", () -> drive.getModules()[1].getAngle().getRadians() + (DriverStation.getAlliance().get().equals(Alliance.Red) ? Math.PI : 0), null);
        builder.addDoubleProperty("Front Right Velocity", () -> drive.getModules()[1].getVelocityMetersPerSec(), null);

        builder.addDoubleProperty("Back Left Angle", () -> drive.getModules()[2].getAngle().getRadians() + (DriverStation.getAlliance().get().equals(Alliance.Red) ? Math.PI : 0), null);
        builder.addDoubleProperty("Back Left Velocity", () -> drive.getModules()[2].getVelocityMetersPerSec(), null);

        builder.addDoubleProperty("Back Right Angle", () -> drive.getModules()[3].getAngle().getRadians() + (DriverStation.getAlliance().get().equals(Alliance.Red) ? Math.PI : 0), null);
        builder.addDoubleProperty("Back Right Velocity", () -> drive.getModules()[3].getVelocityMetersPerSec(), null);

        builder.addDoubleProperty("Robot Angle", () -> drive.getRotation().getRadians() + (DriverStation.getAlliance().get().equals(Alliance.Red) ? Math.PI : 0), null);
      }
    });

    SmartDashboard.putData(field);

    autoChooser = AutoChooser.create(
        robotContainer, robotContainer.getDrive(),
        robotContainer.getSuperstructure());
    Shuffleboard.getTab("Testing")
        .add("Auto Program", autoChooser)
        .withSize(6, 3)
        .withPosition(12, 0)
        .withWidget(BuiltInWidgets.kComboBoxChooser);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    field.setRobotPose(robotContainer.getDrive().getPose());
    

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {

    // autoChooser.reset("SmartDashboard/Autonomous/2025Programs");
    // robotContainer.getSuperstructure().setWantedSuperState(SuperState.STOPPED);
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    autoChooser.update();
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    // autonomousCommand = robotContainer.getAutonomousCommand();

    // // schedule the autonomous command (example)
    // if (autonomousCommand != null) {
    // autonomousCommand.schedule();
    // }

    //TODO test if this works or if there is a better way to do it? 
    //robotContainer.getSuperstructure().getIntake().getIntakeIO().moveHopperToPosition(100);

    autoChooser.getSelectedCommand().ifPresent(CommandScheduler.getInstance()::schedule);
    System.out.println(autoChooser.getSelectedCommand());
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}
