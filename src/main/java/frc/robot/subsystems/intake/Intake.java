package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Intake extends SubsystemBase {
    public abstract void setGoalRPM(double rpmGoal);
    public abstract Command startIntake();
    public abstract Command startIdle();
    public abstract Command startReverse(); 
    public abstract Command stop();
    public abstract boolean isAtRPM();
    public abstract Command openIntake();
    public abstract Command closeIntake();
    public abstract Command semiIntake();
}