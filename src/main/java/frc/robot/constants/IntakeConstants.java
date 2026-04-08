package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class IntakeConstants {
    public static double intakeGearRatio = 1;
    public static double intakeIntakeRPM = 1500;
    public static double intakeOuttakeRPM = -1500;
    public static double intakeIdleRPM = 0;
    public static double rpmTol = 50;

    public static double kP = 0.011;
    public static double kI = 0;
    public static double kD = 0;


    /* Gereksiz
    public static double pivotMotorGearRatio = 1.0 / 5.0;
    public static double pivotGearRatio = 1;   // ölçülecek
    public static double pivotBeltRatio = 1;   // ölçülecek
    public static double pivotFullRatio = pivotBeltRatio * pivotGearRatio * pivotMotorGearRatio;
    */
    public static double pivotClosePosition = 0; //zero position
    public static double pivotOpenPositionOffset = 0;
    public static double pivotSemiPositionOffset = 0;

    public static double rollerGearRatio = 2;
    public static int intakeMotor1_ID = 0;
    public static double intakeMotorCoefficient = rollerGearRatio;

    public static int pivotMotor_ID = 0;

    public static CANBus canbus = new CANBus("canivore");

    public static enum PivotStates {
        CLOSED,
        SEMI,
        OPEN
    }
    TalonFXConfiguration configs = new TalonFXConfiguration();
}