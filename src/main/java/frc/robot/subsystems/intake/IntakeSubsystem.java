package frc.robot.subsystems.intake;
 
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
 
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.IntakeConstants.PivotStates;
import static edu.wpi.first.units.Units.*;


import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
public class IntakeSubsystem extends Intake {
 

    private double goalRPM = IntakeConstants.intakeIdleRPM;
    private double goalPosition = IntakeConstants.pivotClosePosition;
    private IntakeConstants.PivotStates pivotState = PivotStates.CLOSED;
    private final TalonFX m_intake = new TalonFX(IntakeConstants.intakeMotor1_ID, IntakeConstants.canbus);
    private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);
    TalonFXConfiguration configs_intake = new TalonFXConfiguration();

    private final TalonFX m_pivot = new TalonFX(IntakeConstants.pivotMotor_ID, IntakeConstants.canbus);
    private final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);
    TalonFXConfiguration configs_pivot = new TalonFXConfiguration();

    public IntakeSubsystem() {
        SmartDashboard.putBoolean("Pivot Control", false);
        //Intake Motor Setup
        setupConfigs_INTAKE();
        /* Retry config apply up to 5 times, report if failure */
        StatusCode status_intake = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
        status_intake = m_intake.getConfigurator().apply(configs_intake);
        if (status_intake.isOK()) break;
        }
        if (!status_intake.isOK()) {
        System.out.println("Could not apply configs, error code: " + status_intake.toString());
        }

        //Pivot Motor Setup
        setupConfigs_PIVOT();

        StatusCode status_pivot = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
        status_pivot = m_pivot.getConfigurator().apply(configs_pivot);
        if (status_pivot.isOK()) break;
        }
        if (!status_pivot.isOK()) {
        System.out.println("Could not apply configs, error code: " + status_pivot.toString());
        }

        m_pivot.setPosition(IntakeConstants.pivotClosePosition);
    }
 
    @Override
    public void periodic() {
        rpmControl();
        pivotControl();
        if (SmartDashboard.getBoolean("IntakeTelemetry", true) || SmartDashboard.getBoolean("AllTelemetry", false)) {
            telemetrize();
        }
    }
 
    public void rpmControl() {
        //goalRPM
        double motorRPM =  -1 * 0 * IntakeConstants.intakeGearRatio;
        m_intake.setControl(m_velocityVoltage.withVelocity(motorRPM/60));
    }
    
    public void pivotControl() {
        switch (pivotState) {
            case CLOSED:
                goalPosition = IntakeConstants.pivotClosePosition;
                break;
            case SEMI:
                goalPosition = (IntakeConstants.pivotClosePosition + IntakeConstants.pivotSemiPositionOffset);
                break;
            case OPEN:
                goalPosition = (IntakeConstants.pivotClosePosition + IntakeConstants.pivotOpenPositionOffset);
                break;
        }
        if (SmartDashboard.getBoolean("PivotControl", false)) {
            m_pivot.setControl(m_positionVoltage.withPosition(goalPosition));
        }
    }
 
    @Override
    public boolean isAtRPM() {
        return isMotorAtRPM(getVelocityMotor1());
    }
 
    private boolean isMotorAtRPM(double velocity) {
        return velocity > (goalRPM - IntakeConstants.rpmTol)
            && velocity < (goalRPM + IntakeConstants.rpmTol);
    }
 
    public void telemetrize() {
        SmartDashboard.putNumber("Intake/RPM1", getVelocityMotor1());
        SmartDashboard.putNumber("Intake/RPM Average", getAverageRPM());
        SmartDashboard.putNumber("Intake/RPM Goal", goalRPM);
        SmartDashboard.putNumber("Intake/PivotPosition", getPositionPivot());
        SmartDashboard.putBoolean("Intake/Is At RPM", isAtRPM());
    }

    public double getVelocityMotor1() { return (m_intake.getVelocity().getValueAsDouble()*60); }
 
    public double getAverageRPM() {
        return (getVelocityMotor1());
    }

    public double getPositionPivot() { return (m_pivot.getPosition().getValueAsDouble()); }
 
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
        return new InstantCommand(() -> pivotState = PivotStates.OPEN);
    }
 
    @Override
    public Command closeIntake() {
        return new InstantCommand(() -> {pivotState = PivotStates.CLOSED; goalRPM = 0;});
    }
 
    @Override
    public Command semiIntake() {
        return new InstantCommand(() -> pivotState = PivotStates.SEMI);
    }

    public void setupConfigs_INTAKE() {
                /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
        configs_intake.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
        configs_intake.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
        configs_intake.Slot0.kP = IntakeConstants.kP; // An error of 1 rotation per second results in 0.11 V output
        configs_intake.Slot0.kI = IntakeConstants.kI; // No output for integrated error
        configs_intake.Slot0.kD = IntakeConstants.kD; // No output for error derivative
        // Peak output of 8 volts
        configs_intake.Voltage.withPeakForwardVoltage(Volts.of(8))
        .withPeakReverseVoltage(Volts.of(-8));

        /* Torque-based velocity does not require a velocity feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
        configs_intake.Slot1.kS = 2.5; // To account for friction, add 2.5 A of static feedforward
        configs_intake.Slot1.kP = 5; // An error of 1 rotation per second results in 5 A output
        configs_intake.Slot1.kI = 0; // No output for integrated error
        configs_intake.Slot1.kD = 0; // No output for error derivative
        // Peak output of 40 A
        configs_intake.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(40))
        .withPeakReverseTorqueCurrent(Amps.of(-40));
    }

    public void setupConfigs_PIVOT() {
        configs_pivot.Slot0.kP = IntakeConstants.PivotkP; // An error of 1 rotation results in 2.4 V output
        configs_pivot.Slot0.kI = IntakeConstants.PivotkI; // No output for integrated error
        configs_pivot.Slot0.kD = IntakeConstants.PivotkD; // A velocity of 1 rps results in 0.1 V output
        configs_pivot.Slot0.kS = 0.04;
        configs_pivot.Slot0.kV = 0.13248;
        configs_pivot.Slot0.kA = 0.002;
        // Peak output of 8 V
        configs_pivot.Voltage.withPeakForwardVoltage(Volts.of(8))
        .withPeakReverseVoltage(Volts.of(-8));

        configs_pivot.Slot1.kP = 60; // An error of 1 rotation results in 60 A output
        configs_pivot.Slot1.kI = 0; // No output for integrated error
        configs_pivot.Slot1.kD = 6; // A velocity of 1 rps results in 6 A output
        // Peak output of 120 A
        configs_pivot.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(40))
        .withPeakReverseTorqueCurrent(Amps.of(-40));
    }
}