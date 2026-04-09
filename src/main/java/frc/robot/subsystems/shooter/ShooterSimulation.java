package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

public class ShooterSimulation extends Shooter {

    public double goalRPM = ShooterConstants.IDLE_RPM;

    private final ShooterCalculator shooterCalc;

    //TODO: PID Configleri constantsa çek(claude yapar)

    public ShooterSimulation(CommandSwerveDrivetrain commandSwerveDrivetrain) {
        shooterCalc = new ShooterCalculator(commandSwerveDrivetrain);
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
        SmartDashboard.putBoolean("Shooter/Is At RPM", isAtRPM());
    }

    // ── RPM Control ───────────────────────────────────────────────────────────
    public void rpmControl() {    }

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

    public double getVelocityShooter1() { return goalRPM;}
    public double getVelocityShooter2() { return goalRPM; }
    public double getVelocityShooter3() { return goalRPM; }
    public double getVelocityShooter4() { return goalRPM; }
    public double getVelocityShooter5() { return goalRPM; }
    public double getVelocityShooter6() { return goalRPM; }
    public double getAverageRPM() {
        return (getVelocityShooter1()
            + getVelocityShooter2()
            + getVelocityShooter3()
            + getVelocityShooter4()
            + getVelocityShooter5()
            + getVelocityShooter6()) / 6.0;
    }


    @Override
    public Command startShoot() {
        return new InstantCommand(() -> {setGoalRPM(0);});
    }

    @Override
    public Command startIdle() {
        return new InstantCommand(() -> {setGoalRPM(0);});
    }

    @Override
    public Command startReverse() {
        return new InstantCommand(() -> {setGoalRPM(0);});
    }
    
    @Override
    public Command startPass() {
        return new InstantCommand(() -> {setGoalRPM(0);});
    }

    @Override
    public Command stop() {
        return new InstantCommand(() -> {setGoalRPM(0);});
    }
}