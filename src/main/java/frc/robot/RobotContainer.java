package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.camera1Name;
import static frc.robot.subsystems.vision.VisionConstants.camera2Name;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelight;
import lombok.Getter;

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
        @SuppressWarnings("unused")
        private final Vision vision;
        private final @Getter Drive drive;
        private final @Getter Indexer indexer;
        private final @Getter Intake intake;
        private final @Getter Shooter shooter;
        // private final Climber climber;

        private final Superstructure superstructure;

        // Controllers
        private final XboxController controller = new XboxController(0);
        private final XboxController buttonboard = new XboxController(1);

        private static double factor = 1;
        
        // private final UsbCamera usbCam;

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
                // climber = new Climber(new ClimberIOTalonFX());

                // LEDs = new LEDs();
                

                //Cameras are disabled bc there is no field for this branch
                vision = new Vision(
                                drive::addVisionMeasurement,
                                drive);
                                // new VisionIOLimelight(camera0Name, drive::getRotation),
                                // new VisionIOLimelight(camera1Name, drive::getRotation),
                                // new VisionIOLimelight(camera2Name, drive::getRotation));

                superstructure = new Superstructure(this, drive, indexer, intake, shooter);

                // Configure the button bindings
                configureButtonBindings();

                // usbCam = CameraServer.startAutomaticCapture(0);
                // usbCam.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
                // usbCam.setResolution(1280, 720);
                // usbCam.setFPS(60);
                // usbCam.setPixelFormat(PixelFormat.kMJPEG);
                
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

               

                // Trigger ClimbOnX = new Trigger(() -> controller.getXButton());

                // FOR EMERGENCIES ONLY!!!!!
                // Trigger StopClimbOnXAndLeftTrigger = new Trigger(() -> controller.getXButton() && controller.getLeftTriggerAxis() > 0.5);

                // ClimbOnX.onTrue(superstructure.setDesiredSuperStateCommand(climber.getCurrentSubstate() == Climber.Substate.UP ? SuperState.CLIMBDOWN : SuperState.CLIMBUP));
                // StopClimbOnXAndLeftTrigger.onTrue(Commands.runOnce(() -> climber.setDesiredSubstate(Climber.Substate.STOPPED)));

                 // kinda stupid but it works
                double tempSpeed = 0.35;

                // x y flipped 4funsies
                drive.setDefaultCommand(
                                DriveCommands.joystickDrive(
                                                drive,
                                                () -> -controller.getLeftY() * tempSpeed,
                                                () -> -controller.getLeftX() * tempSpeed,
                                                () -> -controller.getRightX()));
                
                Trigger resetPoseTrigger = new Trigger(() -> controller.getRawButton(8));
                resetPoseTrigger.onTrue(
                        Commands.runOnce(
                                () -> drive.setPose(
                                        new Pose2d(
                                                drive.getPose().getX(),
                                                drive.getPose().getY(),
                                                FieldConstants.getAlliance() == Alliance.Blue ? Rotation2d.fromDegrees(0) : Rotation2d.fromDegrees(180))),drive)
                                                .ignoringDisable(true));

                Trigger IntakeOnRightBumper = new Trigger(() -> controller.getRightBumperButton());

                IntakeOnRightBumper.onTrue(superstructure.setDesiredSuperStateCommand(SuperState.INTAKING)
                                .alongWith(superstructure.MoveArmToPosition(1.4)));
                IntakeOnRightBumper.onFalse(superstructure.setDesiredSuperStateCommand(SuperState.DRIVING));
                
                // Trigger AutoAlignPreShooting  = new Trigger(() -> 
                //                                 (controller.getRightBumperButton() && 
                //                                 !DriveCommands.isAimedAtTarget(drive, 
                //                                 () -> drive.getPose().relativeTo(FieldConstants.getHubPose().toPose2d())
                //                                 .getTranslation().getAngle().plus(Rotation2d.k180deg))));
                // AutoAlignPreShooting.onTrue(new InstantCommand(() -> RobotState.getInstance().setAutoAiming(true)));
                // AutoAlignPreShooting.whileTrue(superstructure.setDesiredSuperStateCommand(SuperState.SHOOTINGPREPARE)
                //                 .alongWith(superstructure.AimShooting(controller, () -> FieldConstants.getHubPose().toPose2d())));
                // // AutoAlignPreShooting.onFalse(new InstantCommand(() -> RobotState.getInstance().setAutoAiming(false)).alongWith(superstructure.setDesiredSuperStateCommand(SuperState.DRIVING)));

                // Trigger AutoAlignThenShootTrigger = new Trigger(() -> 
                //                                 (controller.getRightBumperButton() && 
                //                                 DriveCommands.isAimedAtTarget(drive, 
                //                                 () -> drive.getPose().relativeTo(FieldConstants.getHubPose().toPose2d())
                //                                 .getTranslation().getAngle().plus(Rotation2d.k180deg))));
                // AutoAlignThenShootTrigger.whileTrue(superstructure.AimShooting(controller, () -> FieldConstants.getHubPose().toPose2d())
                //                 .alongWith(superstructure.setDesiredSuperStateCommand(SuperState.SHOOTING)));
                

                // //No longer automatically 
                // // AutoAlignThenShootTrigger.whileTrue(superstructure.MoveArmToPosition(0)
                // //                 .withDeadline(Commands.waitSeconds(0.6)).andThen(superstructure.MoveArmToPosition(1.3).withDeadline(Commands.waitSeconds(0.6))).repeatedly());
                // AutoAlignThenShootTrigger.onFalse(superstructure.setDesiredSuperStateCommand(SuperState.DRIVING)
                //                                 .alongWith(new InstantCommand(() -> RobotState.getInstance().setAutoAiming(false))));


                Trigger SlowTurnOnLeftTrigger = new Trigger(() -> (controller.getLeftTriggerAxis() > 0.5));
                SlowTurnOnLeftTrigger.onTrue(Commands.runOnce(() -> factor = .4));
                SlowTurnOnLeftTrigger.onFalse(Commands.runOnce(() -> factor = 1));

                Trigger ShootOnRightTrigger = new Trigger(
                                () -> (controller.getRightTriggerAxis() > 0.5 && !(controller.getRawButton(5))));

                ShootOnRightTrigger.whileTrue(superstructure.setDesiredSuperStateCommand(SuperState.SHOOTING).andThen(new RepeatCommand(moveArmRepeatedly())));
                ShootOnRightTrigger.onFalse(superstructure.setDesiredSuperStateCommand(SuperState.DRIVING).andThen(superstructure.MoveArmToPosition(1.5)));

                Trigger SlowShootOnDPadUp = new Trigger(
                                () -> (controller.getPOV() == 0 && !(controller.getRawButton(5))));

                SlowShootOnDPadUp.whileTrue(Commands.runOnce(() -> shooter.setShooterVelocity(35)).andThen(superstructure.setDesiredSuperStateCommand(SuperState.SHOOTING)).andThen(new RepeatCommand(moveArmRepeatedly())));
                SlowShootOnDPadUp.onFalse(superstructure.setDesiredSuperStateCommand(SuperState.DRIVING).andThen(superstructure.MoveArmToPosition(1.5)).andThen(Commands.runOnce(() -> shooter.setShooterVelocity(70))));

                Trigger ShootOnBButtonWOArm = new Trigger(
                                () -> (controller.getBButton() && !(controller.getRawButton(5))));

                ShootOnBButtonWOArm.whileTrue(superstructure.setDesiredSuperStateCommand(SuperState.SHOOTING));
                ShootOnBButtonWOArm.onFalse(superstructure.setDesiredSuperStateCommand(SuperState.DRIVING));

                Trigger ShootWhileIndexerOutOnBothBumpers = new Trigger(() -> ((controller.getRightBumperButton() || controller.getBButton()) && controller.getLeftBumperButton()));

                ShootWhileIndexerOutOnBothBumpers.onTrue(superstructure.setDesiredSuperStateCommand(SuperState.SHOOTINGWHILEINDEXEROUT));
                ShootWhileIndexerOutOnBothBumpers.onFalse(superstructure.setDesiredSuperStateCommand(SuperState.SHOOTING));

                Trigger CleaningStateTrigger = new Trigger(() -> (controller.getAButton()));

                CleaningStateTrigger.onTrue(superstructure.setDesiredSuperStateCommand(SuperState.CLEANING));
                CleaningStateTrigger.onFalse(superstructure.setDesiredSuperStateCommand(SuperState.DRIVING));
                
                // //TODO Placeholder button. DO NOT DEPLOY 
                // Trigger FerryShotOnBButtonAndLefTrigger = new Trigger(() -> controller.getBButton() && controller.getLeftTriggerAxis() > 0.5);
                // FerryShotOnBButtonAndLefTrigger.whileTrue((superstructure.AimShooting(controller, () -> FieldConstants.getFerryPose(drive.getPose().getTranslation()).toPose2d())).alongWith(superstructure.setDesiredSuperStateCommand(SuperState.SHOOTING)));
                // FerryShotOnBButtonAndLefTrigger.onFalse(new InstantCommand(() -> RobotState.getInstance().setAutoAiming(false)));

                // Trigger AlignHubOnRightBumper = new Trigger(() -> controller.getRawButton(6));
                // AlignHubOnRightBumper.whileTrue(superstructure.AimShooting(controller, () -> FieldConstants.getHubPose().toPose2d())); 
                // AlignHubOnRightBumper.onFalse(new InstantCommand(() -> RobotState.getInstance().setAutoAiming(false)));

                // Trigger WheelRadiusCharacterization = new Trigger(() -> buttonboard.getRawButton(4));
                // WheelRadiusCharacterization.whileTrue(DriveCommands.wheelRadiusCharacterization(drive));
                // WheelRadiusCharacterization.onFalse(superstructure.setDesiredSuperStateCommand(SuperState.DRIVING));


                Trigger IncreaseVelocityBy1 = new Trigger(() -> buttonboard.getRawButton(3)); // Top left button on buttonboard
                IncreaseVelocityBy1.onTrue(Commands.runOnce(() -> shooter.adjustShooterChangeVelocity(1)));

                Trigger DecreaseVelocityBy1 = new Trigger(() -> buttonboard.getRawButton(1)); // Bottom left button on buttonboard
                DecreaseVelocityBy1.onTrue(Commands.runOnce(() -> shooter.adjustShooterChangeVelocity(-1)));

                Trigger IncreaseVelocityBy5 = new Trigger(() -> buttonboard.getRawButton(4)); // 2nd to Top left button on buttonboard
                IncreaseVelocityBy5.onTrue(Commands.runOnce(() -> shooter.adjustShooterChangeVelocity(5)));

                Trigger DecreaseVelocityBy5 = new Trigger(() -> buttonboard.getRawButton(2)); // 2nd to Bottom left button on buttonboard
                DecreaseVelocityBy5.onTrue(Commands.runOnce(() -> shooter.adjustShooterChangeVelocity(-5)));

                // Trigger IncreaseVelocityByOneTenth = new Trigger(() -> buttonboard.getRawButton(4)); // 2nd to top left button
                // IncreaseVelocityByOneTenth.onTrue(Commands.runOnce(() -> shooter.setShooterChangeVelocity(0)));

                // Trigger DecreaseVelocityByOneTenth = new Trigger(() -> buttonboard.getRawButton(2)); // 2nd to bottom left button
                // DecreaseVelocityByOneTenth.onTrue(Commands.runOnce(() -> shooter.adjustShooterChangeVelocity(10)));

                Trigger MoveIntakeArmOut = new Trigger(() -> buttonboard.getLeftTriggerAxis() > 0.5);
                MoveIntakeArmOut.onTrue(superstructure.MoveArmToPosition(1.5));
                Trigger MoveIntakeArmIn = new Trigger(() -> buttonboard.getRightTriggerAxis() > 0.5);
                MoveIntakeArmIn.onTrue(superstructure.MoveArmToPosition(0));
                
                Trigger ApplyArmVoltageIn = new Trigger(() -> buttonboard.getRawButton(6));
                ApplyArmVoltageIn.onTrue(superstructure.SetArmVoltage(-2));
                ApplyArmVoltageIn.onFalse(superstructure.SetArmVoltage(0));
                Trigger ApplyArmVoltageOut = new Trigger(() -> buttonboard.getRawButton(5));
                ApplyArmVoltageOut.onTrue(superstructure.SetArmVoltage(2));
                ApplyArmVoltageOut.onFalse(superstructure.SetArmVoltage(0));

                Trigger resetIntakePosition = new Trigger(() -> buttonboard.getRawButton(8)); // THIS IS START BUTTON
                resetIntakePosition.onTrue(superstructure.SetArmEncoderPosition(0));

        }

        public Superstructure getSuperstructure() {
                return superstructure;
        }

        public static double getFactor() {
                return factor;
        }

        public Command moveArmRepeatedly() {
                return superstructure.MoveArmToPosition(0.5).andThen(new WaitCommand(1)).andThen(superstructure.MoveArmToPosition(1.5)).andThen(new WaitCommand(1));
        }
}
