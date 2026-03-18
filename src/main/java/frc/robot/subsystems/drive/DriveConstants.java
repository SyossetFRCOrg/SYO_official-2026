package frc.robot.subsystems.drive;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.LoggedTunableNumber;

import static edu.wpi.first.units.Units.Degrees;


import edu.wpi.first.math.controller.PIDController;


public class DriveConstants {
    public static final Distance shooterSideOffset = Units.Inches.of(4.5);
    public static final double maxAngularRate = Units.RotationsPerSecond.of(0.75).in(Units.RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    public static final Angle epsilonAngleToGoal = Degrees.of(0.1);

    public static final PIDController rotationController = getRotationController();

    private static final PIDController getRotationController() {
        PIDController controller = new PIDController(2.0, 0.0, 0.0);
        controller.enableContinuousInput(-Math.PI, Math.PI);
        return controller;
    }


    // DriveCommands LoggedTunableNumbers moved
    public static final double ROTATION_TOLERANCE = 5.5; // degrees
    public static final double ANGLE_KP = 10.0;
    public static final double ANGLE_KD = 0.2;
    public static final double ANGLE_MAX_VELOCITY = TunerConstants.driveConfig.maxAngularVelocity() * 1.5;
    public static final double ANGLE_MAX_ACCELERATION = TunerConstants.driveConfig.maxAngularAcceleration() * 1.5;
    public static final double FF_START_DELAY = 2.0; // Secs
    public static final double FF_RAMP_RATE = 0.1; // Volts/Sec
    public static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
    public static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

}
