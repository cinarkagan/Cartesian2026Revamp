package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class AutoShootCommand extends Command{
    private final ShooterSubsystem m_shooter;
    private final FeederSubsystem m_feeder;
    private boolean isFeedTimeSet = false;

    long feedStartTime = 0;
    public AutoShootCommand(ShooterSubsystem m_shooter, FeederSubsystem m_feeder) {
        this.m_shooter = m_shooter;
        this.m_feeder = m_feeder;
    }

    @Override
    public void initialize() {
        m_shooter.customMethod(); //revert to shoot
    }

    @Override
    public void execute() {
        if (m_shooter.isAtRPM()) {
            m_feeder.startFeedingMethod();
            if (!isFeedTimeSet) {
                feedStartTime = System.currentTimeMillis();
                isFeedTimeSet = true;
            }
        }
    }

    @Override
    public boolean isFinished() {
        if (isFeedTimeSet){
            if ((System.currentTimeMillis()-feedStartTime)>ShooterConstants.maxShootTime*1000) {return true;}
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_feeder.stop();
        m_shooter.startIdle();
    }
}
