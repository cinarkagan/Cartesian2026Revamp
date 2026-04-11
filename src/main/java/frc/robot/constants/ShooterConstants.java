package frc.robot.constants;

import com.ctre.phoenix6.CANBus;

public class ShooterConstants {
    public static CANBus canbus = new CANBus("rio");

    public static int shooter1_ID = 57; 
    public static boolean shooter1_reversed = true; ////
    public static int shooter2_ID = 9; //   
     public static boolean shooter2_reversed = false;
    public static int shooter3_ID = 51; //
    public static boolean shooter3_reversed = false;
    public static int shooter4_ID = 55; 
    public static boolean shooter4_reversed = false; ////
    public static int shooter5_ID = 52; //
    public static boolean shooter5_reversed = false; ////
    public static int shooter6_ID = 56; 
    public static boolean shooter6_reversed = false; ////

    public static double IDLE_RPM = 200; //bunu değiştirerelk hedef rpmi belirle
    public static double IDLE_RPM_DIST = 4; //in units of meters
    public static double IDLE_RPM_DIST_PASS = 4; //in units of meters
    public static double IDLE_RPM_MAX = 300;
    public static double IDLE_RPM_MIN = 100;

    public static double SHOOT_RPM = 1000;

    public static double LOW_POWER_RPM = 100;
    public static double REVERSE_RPM = -IDLE_RPM;
    public static double rpmTol = 100;
    
    public static double flywheelGearRatio = 1.5;
    public static double hoodGearReduction = 1.5;

    public static double MIN_FLYWHEEL_SPEED = 100;
    public static double MAX_FLYWHEEL_SPEED = 300;

    public static double kP = 0.008; //pid
    public static double kI = 0;
    public static double kD = 0.0075;
    
    public static double kFF = 1.0 / 5700.0; // NEO free speed feedforward (~0.000175)

    //Shoot Poses
    public static double autoShootRPM = 100;

    public static double maxShootTime = 7.5; //seconds & for pass too
    public static double maxWaitTime = 2;
}
