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

    public double goalRPM = ShooterConstants.IDLE_RPM;


    private final ShooterCalculator shooterCalc;

    //TODO: PID Configleri constantsa çek(claude yapar)

    public ShooterSubsystem(CommandSwerveDrivetrain commandSwerveDrivetrain) {
        shooterCalc = new ShooterCalculator(commandSwerveDrivetrain);

        // Shooter 1
        configShooter1.inverted(ShooterConstants.shooter1_reversed).idleMode(IdleMode.kBrake);
        configShooter1.encoder.positionConversionFactor(1000).velocityConversionFactor(1000);
        configShooter1.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);
        shooter1.configure(configShooter1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Shooter 2
        configShooter2.inverted(ShooterConstants.shooter2_reversed).idleMode(IdleMode.kBrake);
        configShooter2.encoder.positionConversionFactor(1000).velocityConversionFactor(1000);
        configShooter2.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);
        shooter2.configure(configShooter2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Shooter 3
        configShooter3.inverted(ShooterConstants.shooter3_reversed).idleMode(IdleMode.kBrake);
        configShooter3.encoder.positionConversionFactor(1000).velocityConversionFactor(1000);
        configShooter3.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);
        shooter3.configure(configShooter3, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Shooter 4
        configShooter4.inverted(ShooterConstants.shooter4_reversed).idleMode(IdleMode.kBrake);
        configShooter4.encoder.positionConversionFactor(1000).velocityConversionFactor(1000);
        configShooter4.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);
        shooter4.configure(configShooter4, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Shooter 5
        configShooter5.inverted(ShooterConstants.shooter5_reversed).idleMode(IdleMode.kBrake);
        configShooter5.encoder.positionConversionFactor(1000).velocityConversionFactor(1000);
        configShooter5.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);
        shooter5.configure(configShooter5, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Shooter 6
        configShooter6.inverted(ShooterConstants.shooter6_reversed).idleMode(IdleMode.kBrake);
        configShooter6.encoder.positionConversionFactor(1000).velocityConversionFactor(1000);
        configShooter6.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);
        shooter6.configure(configShooter6, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        rpmControl();
        if (SmartDashboard.getBoolean("ShooterTelemetry", false)||SmartDashboard.getBoolean("AllTelemetry", false)) {telemetrize();}
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
        SmartDashboard.putBoolean("Shooter/Is At RPM", isAtRPM());
    }

    // ── RPM Control ───────────────────────────────────────────────────────────
    public void rpmControl() {
        double motorRPM = goalRPM * ShooterConstants.flywheelGearRatio;
        shooter1.getClosedLoopController().setSetpoint(motorRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        shooter2.getClosedLoopController().setSetpoint(motorRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        shooter3.getClosedLoopController().setSetpoint(motorRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        shooter4.getClosedLoopController().setSetpoint(motorRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        shooter5.getClosedLoopController().setSetpoint(motorRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        shooter6.getClosedLoopController().setSetpoint(motorRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
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
        return new InstantCommand(() -> {setGoalRPM(shooterCalc.calculateFlywheelShootRPMFromCurrentPose());});
    }

    @Override
    public Command startIdle() {
        return new InstantCommand(() -> {setGoalRPM(shooterCalc.calculateRestFlywheelSpeedFromCurrentPose());});
    }

    @Override
    public Command startReverse() {
        return new InstantCommand(() -> {setGoalRPM(ShooterConstants.REVERSE_RPM);});
    }

    @Override
    public Command stop() {
        return new InstantCommand(() -> {setGoalRPM(0);});
    }
}