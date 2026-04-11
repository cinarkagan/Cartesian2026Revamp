package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.FeederConstants;
import frc.robot.utils.AllStates;
import frc.robot.utils.AllStates.FeederStates;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;

public class FeederSubsystem extends Feeder {

    private final SparkMax feeder1 = new SparkMax(FeederConstants.feeder1_ID, MotorType.kBrushless);
    private final SparkMax feeder2 = new SparkMax(FeederConstants.feeder2_ID, MotorType.kBrushless);

    SparkMaxConfig configFeeder1 = new SparkMaxConfig();
    SparkMaxConfig configFeeder2 = new SparkMaxConfig();

    public double goalRPM = FeederConstants.IDLE_RPM;

    public FeederSubsystem() {
        // Feeder 1
        configFeeder1.inverted(FeederConstants.feeder1_reversed).idleMode(IdleMode.kCoast);
        configFeeder1.encoder.positionConversionFactor(1).velocityConversionFactor(1);
        configFeeder1.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(FeederConstants.kP, FeederConstants.kI, FeederConstants.kD).velocityFF(FeederConstants.kFF);
        configFeeder1.smartCurrentLimit(40);
        feeder1.configure(configFeeder1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Feeder 2
        configFeeder2.follow(feeder1, true).idleMode(IdleMode.kCoast);
        configFeeder2.smartCurrentLimit(40);
        feeder2.configure(configFeeder2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SmartDashboard.putBoolean("Feeder/Feeder1", true);
    }

    @Override
    public void periodic() {
        rpmControl();
        if (SmartDashboard.getBoolean("FeederTelemetry", true) || SmartDashboard.getBoolean("AllTelemetry", false)) {
            telemetrize();
        }
    }

    public void rpmControl() {
        double motorRPM = goalRPM * FeederConstants.feederGearRatio;
        if (true) {
            feeder1.getClosedLoopController().setSetpoint(motorRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        } else {
            feeder1.set(0);
        } 
    }

    @Override
    public boolean isAtRPM() {
        return isMotorAtRPM(getVelocityFeeder1())
            && isMotorAtRPM(getVelocityFeeder2());
    }

    private boolean isMotorAtRPM(double velocity) {
        return velocity > (goalRPM - FeederConstants.rpmTol)
            && velocity < (goalRPM + FeederConstants.rpmTol);
    }
    
    public void telemetrize() {
        SmartDashboard.putNumber("Feeder/RPM1", getVelocityFeeder1());
        SmartDashboard.putNumber("Feeder/RPM2", getVelocityFeeder2());
        SmartDashboard.putNumber("Feeder/RPM Average", getAverageRPM());
        SmartDashboard.putNumber("Feeder/RPM Goal", getRPMGoal());
        SmartDashboard.putBoolean("Feeder/Is At RPM", isAtRPM());
    }

    public double getRPMGoal()      { return goalRPM; }
    @Override    
    public void setGoalRPM(double goal)      {goalRPM=goal; }

    public double getRPMFeeder()    { return getAverageRPM() / FeederConstants.feederGearRatio; }

    public double getVelocityFeeder1() { return feeder1.getEncoder().getVelocity(); }
    public double getVelocityFeeder2() { return feeder2.getEncoder().getVelocity(); }

    public double getAverageRPM() {
        return (getVelocityFeeder1() + getVelocityFeeder2()) / 2.0;
    }

    public Command waitForFeeder() {
        return new WaitUntilCommand(this::isAtRPM);
    }
    @Override
    public Command startFeeding() {
        return new InstantCommand(() -> {setGoalRPM(FeederConstants.FEED_RPM);});
    }
    @Override
    public Command startIdle() {
        return new InstantCommand(() -> {setGoalRPM(FeederConstants.IDLE_RPM);});
    }
    @Override
    public Command stop() {
        return new InstantCommand(() -> {setGoalRPM(0);});
    }
    @Override
    public Command startReverse() {
        return new InstantCommand(() -> {setGoalRPM(FeederConstants.REVERSE_RPM);});
    }
    public void startFeedingMethod() {
        setGoalRPM(FeederConstants.FEED_RPM);
    }

    public void startIdleMethod() {
        setGoalRPM(FeederConstants.IDLE_RPM);
    }

    public void startReverseMethod() {
        setGoalRPM(FeederConstants.REVERSE_RPM);
    }

    public void stopMethod() {
        setGoalRPM(0);
    }
}