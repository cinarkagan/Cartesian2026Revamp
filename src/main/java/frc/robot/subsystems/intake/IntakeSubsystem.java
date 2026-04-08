package frc.robot.subsystems.intake;
 
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
 
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.IntakeConstants;
 
public class IntakeSubsystem extends Intake {
 
    private final SparkMax intakeMotor1 = new SparkMax((int) IntakeConstants.intakeMotor1_ID, MotorType.kBrushless);
    private final SparkMax intakeMotor2 = new SparkMax((int) IntakeConstants.intakeMotor2_ID, MotorType.kBrushless);
 
    SparkMaxConfig configMotor1 = new SparkMaxConfig();
    SparkMaxConfig configMotor2 = new SparkMaxConfig();
 
    private double goalRPM = IntakeConstants.intakeIdleRPM;
    private double pivotAngle = 0;
 
    public IntakeSubsystem() {
        // Motor 1
        configMotor1.inverted(IntakeConstants.intakeMotor1_Reversed).idleMode(IdleMode.kBrake);
        configMotor1.encoder.positionConversionFactor(1000).velocityConversionFactor(1000);
        configMotor1.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD);
        intakeMotor1.configure(configMotor1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
 
        // Motor 2
        configMotor2.inverted(IntakeConstants.intakeMotor2_Reversed).idleMode(IdleMode.kBrake);
        configMotor2.encoder.positionConversionFactor(1000).velocityConversionFactor(1000);
        configMotor2.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD);
        intakeMotor2.configure(configMotor2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
 
    @Override
    public void periodic() {
        rpmControl();
        if (SmartDashboard.getBoolean("IntakeTelemetry", false) || SmartDashboard.getBoolean("AllTelemetry", false)) {
            telemetrize();
        }
    }
 
    public void rpmControl() {
        double motorRPM = goalRPM * IntakeConstants.intakeGearRatio;
        intakeMotor1.getClosedLoopController().setSetpoint(motorRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        intakeMotor2.getClosedLoopController().setSetpoint(motorRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }
 
    @Override
    public boolean isAtRPM() {
        return isMotorAtRPM(getVelocityMotor1())
            && isMotorAtRPM(getVelocityMotor2());
    }
 
    private boolean isMotorAtRPM(double velocity) {
        return velocity > (goalRPM - IntakeConstants.rpmTol)
            && velocity < (goalRPM + IntakeConstants.rpmTol);
    }
 
    public void telemetrize() {
        SmartDashboard.putNumber("Intake/RPM1", getVelocityMotor1());
        SmartDashboard.putNumber("Intake/RPM2", getVelocityMotor2());
        SmartDashboard.putNumber("Intake/RPM Average", getAverageRPM());
        SmartDashboard.putNumber("Intake/RPM Goal", goalRPM);
        SmartDashboard.putBoolean("Intake/Is At RPM", isAtRPM());
    }
 
    public double getVelocityMotor1() { return intakeMotor1.getEncoder().getVelocity(); }
    public double getVelocityMotor2() { return intakeMotor2.getEncoder().getVelocity(); }
 
    public double getAverageRPM() {
        return (getVelocityMotor1() + getVelocityMotor2()) / 2.0;
    }
 
    public Command waitForIntake() {
        return new WaitUntilCommand(this::isAtRPM);
    }
 
    @Override
    public void setGoalRPM(double rpmGoal) {
        goalRPM = rpmGoal;
    }
 
    @Override
    public Command startIntake() {
        return new InstantCommand(() -> setGoalRPM(IntakeConstants.intakeIntakeRPM));
    }
 
    @Override
    public Command startIdle() {
        return new InstantCommand(() -> setGoalRPM(IntakeConstants.intakeIdleRPM));
    }
 
    @Override
    public Command startReverse() {
        return new InstantCommand(() -> setGoalRPM(IntakeConstants.intakeOuttakeRPM));
    }
 
    @Override
    public Command stop() {
        return new InstantCommand(() -> setGoalRPM(0));
    }
 
 
    @Override
    public Command openIntake() {
        return new InstantCommand(() -> pivotAngle = IntakeConstants.pivotOpenPosition);
    }
 
    @Override
    public Command closeIntake() {
        return new InstantCommand(() -> pivotAngle = IntakeConstants.pivotClosePosition);
    }
 
    @Override
    public Command semiIntake() {
        return new InstantCommand(() -> pivotAngle = IntakeConstants.pivotSemiPosition);
    }
}