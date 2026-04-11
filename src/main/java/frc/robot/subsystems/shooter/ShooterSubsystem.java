package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterUtils.ShooterCalculator;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.utils.AllStates;
import frc.robot.utils.AllStates.ShooterStates;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;

public class ShooterSubsystem extends Shooter {

    private final SparkMax shooter1 = new SparkMax(ShooterConstants.shooter1_ID, MotorType.kBrushless);
    private final SparkMax shooter2 = new SparkMax(ShooterConstants.shooter2_ID, MotorType.kBrushless);
    private final SparkMax shooter3 = new SparkMax(ShooterConstants.shooter3_ID, MotorType.kBrushless);
    private final SparkMax shooter4 = new SparkMax(ShooterConstants.shooter4_ID, MotorType.kBrushless);
    private final SparkMax shooter5 = new SparkMax(ShooterConstants.shooter5_ID, MotorType.kBrushless);
    private final SparkMax shooter6 = new SparkMax(ShooterConstants.shooter6_ID, MotorType.kBrushless);

    SparkMaxConfig configShooter1 = new SparkMaxConfig();
    SparkMaxConfig configShooter2 = new SparkMaxConfig();
    SparkMaxConfig configShooter3 = new SparkMaxConfig();
    SparkMaxConfig configShooter4 = new SparkMaxConfig();
    SparkMaxConfig configShooter5 = new SparkMaxConfig();
    SparkMaxConfig configShooter6 = new SparkMaxConfig();

    public double goalRPM = 0;
    public boolean customRPM = false;


    //private final ShooterCalculator shooterCalc;

    //TODO: PID Configleri constantsa çek(claude yapar)

    public ShooterSubsystem(){//CommandSwerveDrivetrain commandSwerveDrivetrain) {
        //shooterCalc = new ShooterCalculator(commandSwerveDrivetrain);

        // Shooter 1 — leader motor, runs closed-loop velocity
        configShooter1.inverted(ShooterConstants.shooter1_reversed).idleMode(IdleMode.kCoast);
        configShooter1.encoder.positionConversionFactor(1).velocityConversionFactor(1);
        configShooter1.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD)
            .velocityFF(ShooterConstants.kFF);
        configShooter1.smartCurrentLimit(30);
        shooter1.configure(configShooter1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        //sol

        // Shooters 2-6 — followers of shooter1.
        // invert=true because shooter1 is reversed while followers are not,
        // so they need to oppose the leader's output to spin in the same physical direction.
        // Verify on the robot: if followers spin backwards, change the invert argument to false.
        configShooter2.follow(shooter1, false).idleMode(IdleMode.kCoast);
        configShooter2.smartCurrentLimit(40).voltageCompensation(12);
        shooter2.configure(configShooter2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        configShooter3.follow(shooter1, false).idleMode(IdleMode.kCoast);
        configShooter3.smartCurrentLimit(40).voltageCompensation(12);
        shooter3.configure(configShooter3, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        configShooter4.follow(shooter1, true).idleMode(IdleMode.kCoast);
        configShooter4.smartCurrentLimit(40).voltageCompensation(12);
        shooter4.configure(configShooter4, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        configShooter5.follow(shooter1, true).idleMode(IdleMode.kCoast);
        configShooter5.smartCurrentLimit(40).voltageCompensation(12);
        shooter5.configure(configShooter5, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        configShooter6.follow(shooter1, true).idleMode(IdleMode.kCoast);
        configShooter6.smartCurrentLimit(40).voltageCompensation(12);
        shooter6.configure(configShooter6, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        //Before Telemetry
        SmartDashboard.putBoolean("ShooterTelemetry", true);
        SmartDashboard.putBoolean("Shooter/Shooter1Running", false);
        SmartDashboard.putBoolean("Shooter/Shooter2Running", false);
        SmartDashboard.putBoolean("Shooter/Shooter3Running", false);
        SmartDashboard.putBoolean("Shooter/Shooter4Running", false);
        SmartDashboard.putBoolean("Shooter/Shooter5Running", false);
        SmartDashboard.putBoolean("Shooter/Shooter6Running", false);
        SmartDashboard.putNumber("Shooter/CustomRPM", ShooterConstants.SHOOT_RPM);
        SmartDashboard.putNumber("Shooter/Power", 0.4);

    }

    @Override
    public void periodic() {
        rpmControl();
        if (SmartDashboard.getBoolean("ShooterTelemetry", true)||SmartDashboard.getBoolean("AllTelemetry", false)) {telemetrize();}
    }

    public void telemetrize() {
        SmartDashboard.putNumber("Shooter/RPM1", getVelocityShooter1());
        SmartDashboard.putNumber("Shooter/RPM2", getVelocityShooter2());
        SmartDashboard.putNumber("Shooter/RPM3", getVelocityShooter3());
        SmartDashboard.putNumber("Shooter/RPM4", getVelocityShooter4());
        SmartDashboard.putNumber("Shooter/RPM5", getVelocityShooter5());
        SmartDashboard.putNumber("Shooter/RPM6", getVelocityShooter6());
        SmartDashboard.putNumber("Shooter/RPM Average", getAverageRPM());
        SmartDashboard.putNumber("Shooter/RPM Goal", getRPMGoal());
    }
    
    // ── RPM Control ───────────────────────────────────────────────────────────
    /*public void rpmControl() {
        double motorRPM = goalRPM * ShooterConstants.flywheelGearRatio;
        if (SmartDashboard.getBoolean("Shooter1Running", false)) {shooter1.getClosedLoopController().setSetpoint(motorRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);} 
        shooter1.getClosedLoopController().setSetpoint(motorRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        shooter2.getClosedLoopController().setSetpoint(motorRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        shooter3.getClosedLoopController().setSetpoint(motorRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        shooter4.getClosedLoopController().setSetpoint(motorRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        shooter5.getClosedLoopController().setSetpoint(motorRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        shooter6.getClosedLoopController().setSetpoint(motorRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        
        /*shooter1.getClosedLoopController().setSetpoint(motorRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        shooter2.set(0);
        shooter3.set(0);
        shooter4.set(0);
        shooter5.set(0);
        shooter6.set(0);
    }*/
    public void rpmControl() {
        /*double motorRPM = goalRPM * ShooterConstants.flywheelGearRatio;
        if (customRPM) {
            motorRPM = SmartDashboard.getNumber("Shooter/CustomRPM", ShooterConstants.SHOOT_RPM);
        }

        // Only command the leader; followers mirror it automatically.
        if (motorRPM != 0) {
            shooter1.getClosedLoopController().setSetpoint(motorRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        } else {
            shooter1.set(0);
        }*/
       if (customRPM) {
        shooter1.set(SmartDashboard.getNumber("Shooter/Power", 0.4));
       } else {
        shooter1.set(0);
       }
    }

    @Override
    public boolean isAtRPM() {
        return isMotorAtRPM(getVelocityShooter1())
            && isMotorAtRPM(getVelocityShooter2())
            && isMotorAtRPM(getVelocityShooter3())
            && isMotorAtRPM(getVelocityShooter4())
            && isMotorAtRPM(getVelocityShooter5())
            && isMotorAtRPM(getVelocityShooter6());
    }

    private boolean isMotorAtRPM(double velocity) {
        return velocity > (goalRPM - ShooterConstants.rpmTol)
            && velocity < (goalRPM + ShooterConstants.rpmTol);
    }
    public double getRPMGoal()     { return goalRPM; }
    @Override
    public void setGoalRPM(double rpmGoal) {
        goalRPM = rpmGoal;
    }   
    public double getRPMFlywheel() { return getAverageRPM() / ShooterConstants.flywheelGearRatio; }

    public double getVelocityShooter1() { return shooter1.getEncoder().getVelocity(); }
    public double getVelocityShooter2() { return shooter2.getEncoder().getVelocity(); }
    public double getVelocityShooter3() { return shooter3.getEncoder().getVelocity(); }
    public double getVelocityShooter4() { return shooter4.getEncoder().getVelocity(); }
    public double getVelocityShooter5() { return shooter5.getEncoder().getVelocity(); }
    public double getVelocityShooter6() { return shooter6.getEncoder().getVelocity(); }
    public double getAverageRPM() {
        return (getVelocityShooter1()
            + getVelocityShooter2()
            + getVelocityShooter3()
            + getVelocityShooter4()
            + getVelocityShooter5()
            + getVelocityShooter6()) / 6.0;
    }

    public Command waitForShooter() {
        return new WaitUntilCommand(this::isAtRPM);
    }

    @Override
    public Command startShoot() {
        return new InstantCommand(() -> {setGoalRPM(ShooterConstants.SHOOT_RPM);customRPM = false;});//shooterCalc.calculateRestFlywheelSpeedFromCurrentPose());});
        //return new InstantCommand(() -> {setGoalRPM(shooterCalc.calculateFlywheelShootRPMFromCurrentPose());});
    }

    @Override
    public Command startPass() {
        return new InstantCommand(() -> {setGoalRPM(ShooterConstants.SHOOT_RPM);customRPM = false;});//shooterCalc.calculateRestFlywheelSpeedFromCurrentPose());});
        //return new InstantCommand(() -> {setGoalRPM(shooterCalc.calculateFlywheelPassRPMFromCurrentPose());});
    }

    @Override
    public Command startIdle() {
        return new InstantCommand(() -> {setGoalRPM(ShooterConstants.IDLE_RPM);customRPM = false;});//shooterCalc.calculateRestFlywheelSpeedFromCurrentPose());});
    }

    @Override
    public Command startReverse() {
        return new InstantCommand(() -> {setGoalRPM(ShooterConstants.REVERSE_RPM);customRPM = false;});
    }

    @Override
    public Command stop() {
        return new InstantCommand(() -> {setGoalRPM(0);customRPM = false;});
    }
    public Command custom() {
        return new InstantCommand(() -> {customRPM = true;});
    }
    public void customMethod() {
        customRPM = true;
    }
    public Command customRelease() {
        return new InstantCommand(() -> {customRPM = false;});
    }
    public void startShootMethod() {
        setGoalRPM(ShooterConstants.SHOOT_RPM);
        //setGoalRPM(shooterCalc.calculateFlywheelShootRPMFromCurrentPose()));
    }

    public void startPassMethod() {
        setGoalRPM(ShooterConstants.SHOOT_RPM);
        //setGoalRPM(shooterCalc.calculateFlywheelPassRPMFromCurrentPose());
    }

    public void startIdleMethod() {
        setGoalRPM(ShooterConstants.IDLE_RPM);
        //setGoalRPM(shooterCalc.calculateRestFlywheelSpeedFromCurrentPose()));
    }

    public void startReverseMethod() {
        setGoalRPM(ShooterConstants.REVERSE_RPM);
    }

    public void stopMethod() {
        setGoalRPM(0);
    }
}