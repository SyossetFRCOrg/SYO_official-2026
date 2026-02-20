package frc.robot.subsystems.drive;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;


import static edu.wpi.first.units.Units.Degrees;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;


public class DriveConstants {
    public static final Distance shooterSideOffset = Units.Inches.of(6.0);
    public static final double maxAngularRate = Units.RotationsPerSecond.of(0.75).in(Units.RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    public static final Angle epsilonAngleToGoal = Degrees.of(0.1);

    public static final PIDController rotationController = getRotationController();

    private static final PIDController getRotationController() {
        PIDController controller = new PIDController(2.0, 0.0, 0.0);
        controller.enableContinuousInput(-Math.PI, Math.PI);
        return controller;
    }
}
