package frc.robot.constants;

public class IntakeConstants {
    public static double intakeGearRatio = 1;
    public static double intakeIntakeRPM = 1500;
    public static double intakeOuttakeRPM = -1500;
    public static double intakeIdleRPM = 0;
    public static double rpmTol = 50;

    public static double kP = 0.011;
    public static double kI = 0;
    public static double kD = 0;

    public static double pivotMotorGearRatio = 1.0 / 5.0;
    public static double pivotGearRatio = 1;   // ölçülecek
    public static double pivotBeltRatio = 1;   // ölçülecek
    public static double pivotFullRatio = pivotBeltRatio * pivotGearRatio * pivotMotorGearRatio;
    public static double pivotClosePosition = 0;
    public static double pivotOpenPosition = 0;
    public static double pivotSemiPosition = 0;

    public static int intakeMotor1_ID = 0;
    public static boolean intakeMotor1_Reversed = false;
    public static int intakeMotor2_ID = 0;
    public static boolean intakeMotor2_Reversed = false;
}