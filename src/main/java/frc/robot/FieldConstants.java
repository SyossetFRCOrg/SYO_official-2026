package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import pabeles.concurrency.IntOperatorTask.Min;

public class FieldConstants {
    // https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf All April tag poses are derived from Welded measurements
    // MAY BE POTENTIALLY PROLBEM ALL ANGLE 0. 

    // Measured from central april tag facing driver station
    public static final Pose3d redHubPose = new Pose3d(Units.Inches.of(492.88),Units.Inches.of(158.84), Units.Inches.of(44.25), Rotation3d.kZero);
    public static final Pose3d redOutpostPose = new Pose3d(Units.Inches.of(650.92), Units.Inches.of(291.47), Units.Inches.of(21.75), Rotation3d.kZero);
    public static final Pose3d redTowerPose = new Pose3d(Units.Inches.of(650.90), Units.Inches.of(170.22), Units.Inches.of(21.75), Rotation3d.kZero);

    public static final Pose3d blueHubPose = new Pose3d(Units.Inches.of(158.34), Units.Inches.of(158.84),  Units.Inches.of(44.25), Rotation3d.kZero);
    public static final Pose3d blueOutputPose = new Pose3d(Units.Inches.of(0.30), Units.Inches.of(26.22), Units.Inches.of(21.75), Rotation3d.kZero);
    public static final Pose3d blueTowerPose = new Pose3d(Units.Inches.of(0.32), Units.Inches.of(164.47), Units.Inches.of(21.75), Rotation3d.kZero);

    public static final Pose3d redFerryOutpostPose = new Pose3d(Units.Inches.of(607.8), Units.Inches.of(237.69), Units.Inches.of(0), Rotation3d.kZero);
    public static final Pose3d redFerryDepotPose = new Pose3d(Units.Inches.of(607.8), Units.Inches.of(80), Units.Inches.of(0), Rotation3d.kZero);
    public static final Pose3d blueFerryOutpostPose = new Pose3d(Units.Inches.of(43.42), Units.Inches.of(80), Units.Inches.of(0), Rotation3d.kZero);
    public static final Pose3d blueFerryDepotPose = new Pose3d(Units.Inches.of(43.42), Units.Inches.of(237.69), Units.Inches.of(0), Rotation3d.kZero);

    
    // Gets pose based on current alliance
    // -->
    public static Pose3d getHubePose() {
        return DriverStation.getAlliance().get().equals(Alliance.Red) ? redHubPose : blueHubPose;
    }

    public static void getFerryPose()
    {
        if(DriverStation.getAlliance().get().equals(Alliance.Red))
        {
        }
    }
}