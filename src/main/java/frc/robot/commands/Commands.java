package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class Commands {
    public Command passCommand(CommandSwerveDrivetrain drivetrain, ShooterSubsystem shooterSubsystem, FeederSubsystem feederSubsystem) {
        return new SequentialCommandGroup(
            new FaceToPass(drivetrain).withTimeout(1750),
            new AutoPassCommand(shooterSubsystem, feederSubsystem)
        );
    }
}
