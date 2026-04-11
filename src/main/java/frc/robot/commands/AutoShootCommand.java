package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
public class AutoShootCommand extends Command {
    private final ShooterSubsystem m_shooter;
    private final FeederSubsystem m_feeder;
    private final IntakeSubsystem m_intake;
    
    private final Timer m_timer = new Timer();
    private boolean isFeedTimeSet = false;
    private double feedStartTime = 0;

    public AutoShootCommand(ShooterSubsystem shooter, FeederSubsystem feeder, IntakeSubsystem intake) {
        this.m_shooter = shooter;
        this.m_feeder = feeder;
        this.m_intake = intake;
        // ALWAYS add requirements
        addRequirements(m_shooter, m_feeder, m_intake);
    }

    @Override
    public void initialize() {
        m_timer.restart(); // Starts the clock at 0.0 seconds
        isFeedTimeSet = false;
        m_shooter.customMethod(); // Start shooter motors spinning
    }

    @Override
    public void execute() {
        // Use the timer (which is in seconds, so no need to * 1000)
        if (m_timer.get() >= ShooterConstants.maxWaitTime) {
            m_feeder.startFeedingMethod();
            m_intake.semiIntakeMethod();
            
            if (!isFeedTimeSet) {
                feedStartTime = m_timer.get();
                isFeedTimeSet = true;
            }
        }
    }

    @Override
    public boolean isFinished() {
        // Finish once we've been feeding for long enough
        return isFeedTimeSet && (m_timer.get() - feedStartTime > ShooterConstants.maxShootTime);
    }

    @Override
    public void end(boolean interrupted) {
        m_feeder.stop();
        m_shooter.stop();
        m_intake.stop(); // Don't forget to stop the intake too!
    }
}