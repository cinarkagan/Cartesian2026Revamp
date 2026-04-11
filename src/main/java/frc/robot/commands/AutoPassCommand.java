package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class AutoPassCommand extends Command{
    private final ShooterSubsystem m_shooter;
    private final FeederSubsystem m_feeder;
    private final IntakeSubsystem m_intake;
    private boolean isFeedTimeSet = false;

    long feedStartTime = 0;
    public AutoPassCommand(ShooterSubsystem m_shooter, FeederSubsystem m_feeder, IntakeSubsystem m_intake) {
        this.m_shooter = m_shooter;
        this.m_feeder = m_feeder;
        this.m_intake = m_intake;
        addRequirements(m_feeder,m_shooter);
    }

    @Override
    public void initialize() {
        m_shooter.startPassMethod();
    }

    @Override
    public void execute() {
        if (m_shooter.isAtRPM()) {
            m_intake.semiIntakeMethod();
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
        }        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_feeder.stop();
        m_shooter.startIdle();
    }
}
