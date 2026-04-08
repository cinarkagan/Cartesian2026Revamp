package frc.robot.controllers;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.MachineSubsystem;
import frc.robot.Telemetry;
import frc.robot.constants.TeleopConstants;
import frc.robot.subsystems.intake.IntakeSimulation;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterSimulation;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.utils.Container;

public class Autonomous implements Controller {
    private boolean simulationMode = Container.simulationMode;
    private Telemetry logger;
    private ShooterSimulation shooterSimulation;
    private IntakeSimulation intakeSimulation;
    private CommandSwerveDrivetrain drivetrain;
    private SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(TeleopConstants.MaxSpeed * 0.1).withRotationalDeadband(TeleopConstants.MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private MachineSubsystem machineSubsystem;
    private PathPlannerPath path1;
    private PathPlannerPath path2;
    public Autonomous(Telemetry logger,CommandSwerveDrivetrain drivetrain, MachineSubsystem machineSubsystem) {
        this.logger = logger;
        this.drivetrain = drivetrain;
        //this.joystick = new Joystick(0);
        this.shooterSimulation = new ShooterSimulation(this.drivetrain);
        this.intakeSimulation = new IntakeSimulation();
        this.machineSubsystem = machineSubsystem;
        try {
            initPaths();
        } catch (FileVersionException | IOException | ParseException e) {
            e.printStackTrace();
        }
    }
    public void initPaths() throws FileVersionException, IOException, ParseException {
        path1 = PathPlannerPath.fromPathFile("AutoPathIntake");
        path2 = PathPlannerPath.fromPathFile("AutoP1Shoot");
    }
    @Override
    public void getInitializeFunction() {}

    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(
            AutoBuilder.followPath(path1),
            intakeSimulation.activateIntakeCommand(),
            new WaitCommand(2),
            intakeSimulation.intakeBallWithRandomness(20),
            intakeSimulation.deactivateIntakeCommand(),
            AutoBuilder.followPath(path2)
        );
    }

    @Override
    public void simulationPeriodic() {} 

    @Override
    public void periodic() {}
    
}
