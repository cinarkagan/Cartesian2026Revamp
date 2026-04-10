package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.AllStates;

public abstract class Shooter extends SubsystemBase {
    public abstract void setGoalRPM(double rpmGoal);
    public abstract Command startShoot();
    public abstract Command startPass();
    public abstract Command startIdle();
    public abstract Command startReverse(); 
    public abstract Command stop();
    public abstract boolean isAtRPM();
}