package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.camera1Name;
import static frc.robot.subsystems.vision.VisionConstants.camera2Name;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelight;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // Subsystems
        private final Vision vision;
        private final Drive drive;
        private final Indexer indexer;
        private final Intake intake;
        private final Shooter shooter;

        private final Superstructure superstructure;

        // Controllers
        private final XboxController controller = new XboxController(0);
        private final XboxController buttonboard = new XboxController(1);
        // private final UsbCamera climbCam;

        // private final HttpCamera climberCamera;

        // private final AutoSelector autoSelector = new AutoSelector("Auto");

        // Dashboard inputs
        // private final LoggedDashboardChooser<Command> autoChooser;

        /**
         * The container for the robot. Contains subsystems, IO devices, and commands.
         */
        public RobotContainer() {

                drive = new Drive(
                                new GyroIOPigeon2(),
                                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                                new ModuleIOTalonFX(TunerConstants.FrontRight),
                                new ModuleIOTalonFX(TunerConstants.BackLeft),
                                new ModuleIOTalonFX(TunerConstants.BackRight));
                indexer = new Indexer(new IndexerIOTalonFX());
                intake = new Intake(new IntakeIOTalonFX());
                shooter = new Shooter(new ShooterIOTalonFX());

                // LEDs = new LEDs();

                vision = new Vision(
                                drive::addVisionMeasurement,
                                drive,
                                new VisionIOLimelight(camera0Name, drive::getRotation));

                superstructure = new Superstructure(this, drive, indexer, intake, shooter);

                // Configure the button bindings
                configureButtonBindings();

                // climbCam = CameraServer.startAutomaticCapture();
                // climbCam.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
                // climbCam.setResolution(80, 60);

                // Shuffleboard.getTab("Match")
                // .add(new HttpCamera("ClimberCam",
                // "http://roborio-9016-frc.local:1181/?action=stream"))
                // .withWidget(BuiltInWidgets.kCameraStream)
                // .withSize(4, 3)
                // .withPosition(4, 3);
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link GenericHID} or one of its subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
         * it to a {@link
         * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
         */
        private void configureButtonBindings() {

                // kinda stupid but it works
                double tempSpeed = 0.35;

                // REALLY BAD FIX, DO NOT KEEP THIS!!!!!
                drive.setDefaultCommand(
                                DriveCommands.joystickDrive(
                                                drive,
                                                () -> controller.getLeftY() * tempSpeed,
                                                () -> controller.getLeftX() * tempSpeed,
                                                () -> -controller.getRightX()));
                Trigger resetPoseTrigger = new Trigger(() -> controller.getRawButton(8));
                resetPoseTrigger.onTrue(
                                Commands.runOnce(
                                                () -> drive.setPose(
                                                                new Pose2d(
                                                                                drive.getPose().getX(),
                                                                                drive.getPose().getY(),
                                                                                DriverStation.getAlliance()
                                                                                                .get() == Alliance.Blue
                                                                                                                ? Rotation2d.fromRadians(
                                                                                                                                180)
                                                                                                                : Rotation2d.fromDegrees(
                                                                                                                                0))),
                                                drive)
                                                .ignoringDisable(true));

                // TODO fix error where triggers are not being registered. Something is wrong
                // with the way I set up these triggers because the code does not do anything
                // with them
                Trigger IntakeOnAPressed = new Trigger(() -> controller.getAButton());

                IntakeOnAPressed.onTrue(superstructure.setDesiredSuperStateCommand(SuperState.INTAKING)
                                .andThen(new InstantCommand(() -> System.out.print("A Button Pressed"))));
                IntakeOnAPressed.onFalse(superstructure.setDesiredSuperStateCommand(SuperState.DRIVING));

                Trigger ShootOnBButton = new Trigger(() -> controller.getBButton());

                ShootOnBButton.onTrue(superstructure.setDesiredSuperStateCommand(SuperState.SHOOTING));
                ShootOnBButton.onFalse(superstructure.setDesiredSuperStateCommand(SuperState.DRIVING));

                Trigger AlignOnXRightBumper = new Trigger(() -> controller.getRawButton(6));
                AlignOnXRightBumper.onTrue(superstructure.prepShot(controller, () -> FieldConstants.getHubePose().toPose2d()));
                AlignOnXRightBumper.onFalse(superstructure.setDesiredSuperStateCommand(SuperState.DRIVING));
        }

        public Drive getDrive() {
                return drive;
        }

        public Superstructure getSuperstructure() {
                return superstructure;
        }
}
