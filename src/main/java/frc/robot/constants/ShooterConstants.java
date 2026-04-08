package frc.robot.constants;

import com.ctre.phoenix6.CANBus;

public class ShooterConstants {
    public static CANBus canbus = new CANBus("rio");

    public static int shooter1_ID = 111856;
    public static boolean shooter1_reversed = false;
    public static int shooter2_ID = 25;
    public static boolean shooter2_reversed = false;
    public static int shooter3_ID = 29;
    public static boolean shooter3_reversed = false;
    public static int shooter4_ID = 26;
    public static boolean shooter4_reversed = false;
    public static int shooter5_ID = 27;
    public static boolean shooter5_reversed = false;
    public static int shooter6_ID = 28;
    public static boolean shooter6_reversed = false;

    public static double IDLE_RPM = 1000;
    public static double IDLE_RPM_DIST = 4; //in units of meters
    public static double IDLE_RPM_DIST_PASS = 4; //in units of meters
    public static double IDLE_RPM_MAX = 1700;
    public static double IDLE_RPM_MIN = 800;

    public static double SHOOT_RPM = 1000;

    public static double LOW_POWER_RPM = 500;
    public static double REVERSE_RPM = -IDLE_RPM;
    public static double rpmTol = 100;
    
    public static double flywheelGearRatio = 1.5;
    public static double hoodGearReduction = 1.5;

    public static double MIN_FLYWHEEL_SPEED = 1700;
    public static double MAX_FLYWHEEL_SPEED = 3000;

    public static double kP = 0.011;
    public static double kI = 0;
    public static double kD = 0;

    //Shoot Poses
    public static double autoShootRPM = 1000;

}
