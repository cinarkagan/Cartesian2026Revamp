package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class AutoPassCommand extends Command{
    private final ShooterSubsystem m_shooter;
    private final FeederSubsystem m_feeder;

    long feedStartTime = 0;
    public AutoPassCommand(ShooterSubsystem m_shooter, FeederSubsystem m_feeder) {
        this.m_shooter = m_shooter;
        this.m_feeder = m_feeder;
    }

    @Override
    public void initialize() {
        m_shooter.startPass();
    }

    @Override
    public void execute() {
        if (m_shooter.isAtRPM()) {
            m_feeder.startFeeding();
            if (feedStartTime == 0) {
                feedStartTime = System.currentTimeMillis();
            }
        }
    }

    @Override
    public boolean isFinished() {
        if ((System.currentTimeMillis()-feedStartTime)>ShooterConstants.maxShootTime) {return true;}
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_feeder.stop();
        m_shooter.startIdle();
    }
}
