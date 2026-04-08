package frc.robot.constants;

import com.ctre.phoenix6.CANBus;

public class FeederConstants {
    public static CANBus canbus = new CANBus("rio");

    public static int feeder1_ID = 111857;
    public static boolean feeder1_reversed = false;
    public static int feeder2_ID = 26;
    public static boolean feeder2_reversed = false;

    public static double IDLE_RPM = 0;
    public static double FEED_RPM = 2000;
    public static double REVERSE_RPM = -1000;
    public static double rpmTol = 50;
    
    public static double feederGearRatio = 1;

    public static double kP = 0.011;
    public static double kI = 0;
    public static double kD = 0;

}
