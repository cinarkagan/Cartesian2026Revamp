package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Feeder extends SubsystemBase {
    public abstract void setGoalRPM(double rpmGoal);
    public abstract Command startFeeding();
    public abstract Command startIdle();
    public abstract Command startReverse(); 
    public abstract Command stop();
    public abstract boolean isAtRPM();
}