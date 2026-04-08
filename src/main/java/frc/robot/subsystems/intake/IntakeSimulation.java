package frc.robot.subsystems.intake;

import java.util.Random;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.IntakeConstants;
import frc.robot.utils.Container;

public class IntakeSimulation extends Intake {
    private boolean isActive = false;
    private double goalRPM = 0;
    private double pivotAngle = 0;
    public IntakeSimulation() {}
    
    public void intakeBall(double amout) {Container.fuelCount = Container.fuelCount + amout;}
    public Command intakeBallWithRandomness(double amount) {
        return new InstantCommand(() -> {
        if (isActive) {
            Random random = new Random();
            double percentage = random.nextDouble(100); 
            int amounttrue = (int) (amount * (percentage/100));
            intakeBall(amounttrue);
        }});
    }
    @Override
    public void periodic() {
        
    }

    @Override
    public void setGoalRPM(double rpmGoal) {
        goalRPM = rpmGoal;
    }

    @Override
    public Command startIntake() {
        return new InstantCommand(
            () -> {
                isActive = true;
                goalRPM = IntakeConstants.intakeIntakeRPM;
            }
        );
    }

    @Override
    public Command startIdle() {
        return new InstantCommand(
            () -> {
                isActive = false;
                goalRPM = IntakeConstants.intakeIdleRPM;
            }
        );
    }

    @Override
    public Command startReverse() {
        return new InstantCommand(
            () -> {
                isActive = false;
                goalRPM = IntakeConstants.intakeOuttakeRPM;
            }
        );
    }

    @Override
    public Command stop() {
        return new InstantCommand(
            () -> {
                isActive = false;
                goalRPM = 0;
            }
        );
    }

    @Override
    public boolean isAtRPM() {
        return true;
    }

    @Override
    public Command openIntake() {
        return new InstantCommand(
            () -> {
                pivotAngle = IntakeConstants.pivotOpenPosition;
            }
        );
    }

    @Override
    public Command closeIntake() {
        return new InstantCommand(
            () -> {
                pivotAngle = IntakeConstants.pivotClosePosition;
            }
        );
    }

    @Override
    public Command semiIntake() {
        return new InstantCommand(
            () -> {
                pivotAngle = IntakeConstants.pivotSemiPosition;
            }
        );
    }
}
