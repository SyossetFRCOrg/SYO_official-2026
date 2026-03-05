package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.camera1Name;
import static frc.robot.subsystems.vision.VisionConstants.camera2Name;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIOTalonFX;
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
        private final Climber climber;

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
                climber = new Climber(new ClimberIOTalonFX());

                // LEDs = new LEDs();

                vision = new Vision(
                                drive::addVisionMeasurement,
                                drive,
                                new VisionIOLimelight(camera0Name, drive::getRotation),
                                new VisionIOLimelight(camera1Name, drive::getRotation));

                superstructure = new Superstructure(this, drive, indexer, intake, shooter, climber);

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

               

                Trigger ClimbOnX = new Trigger(() -> controller.getXButton());

                // FOR EMERGENCIES ONLY!!!!!
                Trigger StopClimbOnXAndLeftTrigger = new Trigger(() -> controller.getXButton() && controller.getLeftTriggerAxis() > 0.5);

                ClimbOnX.onTrue(superstructure.setDesiredSuperStateCommand(climber.getCurrentSubstate() == Climber.Substate.UP ? SuperState.CLIMBDOWN : SuperState.CLIMBUP));
                StopClimbOnXAndLeftTrigger.onTrue(Commands.runOnce(() -> climber.setDesiredSubstate(Climber.Substate.STOPPED)));

                 // kinda stupid but it works
                double tempSpeed = 0.35;

                // x y flipped 4funsies
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
                                                DriverStation.getAlliance().get() == Alliance.Blue ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0))),drive)
                                                .ignoringDisable(true));

                Trigger IntakeOnAPressed = new Trigger(() -> controller.getAButton());

                IntakeOnAPressed.onTrue(superstructure.setDesiredSuperStateCommand(SuperState.INTAKING));
                IntakeOnAPressed.onFalse(superstructure.setDesiredSuperStateCommand(SuperState.DRIVING));

                Trigger ShootOnBButton = new Trigger(
                                () -> (controller.getBButton() && !(controller.getRightTriggerAxis() > 0.5)));

                ShootOnBButton.onTrue(superstructure.setDesiredSuperStateCommand(SuperState.SHOOTING));
                ShootOnBButton.onFalse(superstructure.setDesiredSuperStateCommand(SuperState.DRIVING));

                Trigger ShootWhileIndexerOutOnBButtonAndRightTrigger = new Trigger(() -> (controller.getRightTriggerAxis() > 0.5 && controller.getBButton()));

                ShootWhileIndexerOutOnBButtonAndRightTrigger.onTrue(superstructure.setDesiredSuperStateCommand(SuperState.SHOOTINGWHILEINDEXEROUT));

                Trigger IntakeAndIndexWithoutShootingTrigger = new Trigger(() -> (controller.getXButton()));

                IntakeAndIndexWithoutShootingTrigger.onTrue(superstructure.setDesiredSuperStateCommand(SuperState.INTAKINGANDINDEXINGWITHOUTSHOOTING));
                IntakeAndIndexWithoutShootingTrigger.onFalse(superstructure.setDesiredSuperStateCommand(SuperState.DRIVING));
                
                //TODO Placeholder button. DO NOT DEPLOY 
                // Trigger FerryShotOnBButtonAndLefTrigger = new Trigger(() -> controller.getBButton() && controller.getLeftTriggerAxis() > 0.5);
                // FerryShotOnBButtonAndLefTrigger.whileTrue((superstructure.AimShooting(controller, () -> FieldConstants.getFerryPose(drive.getPose().getTranslation()).toPose2d())));

                //TODO consider moving this to buttonboard?
                Trigger moveHopperOutOnDpadUp = new Trigger(() -> buttonboard.getRawButton(6));
                moveHopperOutOnDpadUp.onTrue(superstructure.SetHopperVoltage(4));
                moveHopperOutOnDpadUp.onFalse(superstructure.SetHopperVoltage(0));
                Trigger moveHopperInOnDpadDown = new Trigger(() -> buttonboard.getRawButton(5));
                moveHopperInOnDpadDown.onTrue(superstructure.SetHopperVoltage(-4));
                moveHopperInOnDpadDown.onFalse(superstructure.SetHopperVoltage(0));


                Trigger AlignHubOnRightBumper = new Trigger(() -> controller.getRawButton(6));
                AlignHubOnRightBumper.whileTrue(superstructure.AimShooting(controller, () -> FieldConstants.getHubePose().toPose2d())); 

                Trigger IncreaseVelocityBy1 = new Trigger(() -> buttonboard.getRawButton(3)); // Top left button on buttonboard
                IncreaseVelocityBy1.onTrue(Commands.runOnce(() -> shooter.adjustShooterVoltage(1)));

                Trigger DecreaseVelocityBy1 = new Trigger(() -> buttonboard.getRawButton(1)); // Bottom left button on buttonboard
                DecreaseVelocityBy1.onTrue(Commands.runOnce(() -> shooter.adjustShooterVoltage(-1)));

                Trigger IncreaseVelocityByOneTenth = new Trigger(() -> buttonboard.getRawButton(4)); // 2nd to top left button
                IncreaseVelocityByOneTenth.onTrue(Commands.runOnce(() -> shooter.adjustShooterVoltage(0.1)));

                Trigger DecreaseVelocityByOneTenth = new Trigger(() -> buttonboard.getRawButton(2)); // 2nd to bottom left button
                DecreaseVelocityByOneTenth.onTrue(Commands.runOnce(() -> shooter.adjustShooterVoltage(-0.1)));

                //TODO: match Intake states/command to trigger
                Trigger MoveIntakeArmOut = new Trigger(() -> buttonboard.getLeftTriggerAxis() > 0.5);
                //MoveIntakeArmOut.onTrue(Commands.runOnce(() -> intake.EXTENDMETHOD()));
                Trigger MoveIntakeArmIn = new Trigger(() -> buttonboard.getRightTriggerAxis() > 0.5);
                //MoveIntakeArmIn.onTrue(Commands.runOnce(() -> intake.RETRACTMETHOD()));

        }

        public Drive getDrive() {
                return drive;
        }

        public Superstructure getSuperstructure() {
                return superstructure;
        }
}
