package frc.robot.constants;

import java.io.IOException;
import java.nio.file.Path;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation;

public class VisionConstants {
    public static final Path fieldLayoutPath = Filesystem.getDeployDirectory()
            .toPath()
            .resolve("apriltags")
            .resolve("andymark")
            .resolve("2026-official.json");

    public static AprilTagFieldLayout layout;

    static {
        try {
            layout = new AprilTagFieldLayout(fieldLayoutPath);
        } catch (IOException e) {
            DriverStation.reportError("Failed to load AprilTag layout: " + e.getMessage(), true);
            layout = null;
        }
    }
    public static String camera1Name = "hello";
    public static String camera2Name = "world";

    public static double ambiguityTol = 0.5;

    public static final Transform3d kRobotToCam1 =
        new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0)); //Ayarla

        
    public static final Transform3d kRobotToCam2 =
        new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0)); //Ayarla
}