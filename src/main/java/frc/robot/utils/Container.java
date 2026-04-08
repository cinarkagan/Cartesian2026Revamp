package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Robot;


public class Container {
    public static Pose2d startPose = new Pose2d();
    public static boolean driveEnabled = true;
    public static boolean simulationMode = Robot.isSimulation();
    public static double fuelCount = 0;
    public static void increaseFuel() {
        fuelCount++;
    }
    public static void decreaseFuel() {
        fuelCount--;
    }
}