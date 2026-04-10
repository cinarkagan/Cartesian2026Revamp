package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.TeleopConstants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

/**
 * Commands the robot to a specific X, Y coordinate and Heading (Rotation) 
 * simultaneously using three independent PID loops.
 */
public class DriveAndFaceTheGoal extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.FieldCentric drive;
    
    private final PIDController controllerX;
    private final PIDController controllerY;
    private final PIDController controllerTheta;

    private final double maxDriveRate;
    private final double maxAngularRate;

    public DriveAndFaceTheGoal(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.maxDriveRate = TeleopConstants.MaxSpeed;
        this.maxAngularRate = TeleopConstants.MaxAngularRate;

        // Initialize Translation Controllers
        this.controllerX = new PIDController(DriveConstants.driveP, DriveConstants.driveI, DriveConstants.driveD);
        this.controllerY = new PIDController(DriveConstants.driveP, DriveConstants.driveI, DriveConstants.driveD);
        
        // Initialize Rotation Controller
        this.controllerTheta = new PIDController(DriveConstants.turnP, DriveConstants.turnI, DriveConstants.turnD);
        
        // Configuration
        this.drive = drivetrain.getDriveRequest();

        //Goal Pose Config
        double goalTheta = drivetrain.getThetaToHub();
        double goalPoseX = Math.sin(goalTheta);
        double goalPoseY = Math.cos(goalTheta);

        // Setup Translation Tolerances (Meters)
        controllerX.setTolerance(DriveConstants.toleranceXCM / 100.0);
        controllerY.setTolerance(DriveConstants.toleranceYCM / 100.0);
        controllerX.setSetpoint(goalPoseX);
        controllerY.setSetpoint(goalPoseY);

        // Setup Rotation Tolerance and Range (Radians)
        // Continuous input allows robot to turn -179 to 179 without spinning 358 degrees
        controllerTheta.enableContinuousInput(-Math.PI, Math.PI);
        controllerTheta.setTolerance(
            Math.toRadians(DriveConstants.turnToleranceDeg), 
            Math.toRadians(DriveConstants.turnToleranceDegPerSec)
        );
        controllerTheta.setSetpoint(goalTheta);

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        Pose2d currentPose = drivetrain.getSwerveDriveState().Pose;

        // Calculate Feedback
        double outputX = controllerX.calculate(currentPose.getX());
        double outputY = controllerY.calculate(currentPose.getY()); // Fixed: was controllerX
        double outputTheta = controllerTheta.calculate(currentPose.getRotation().getRadians());

        // Clamp outputs to physical limits
        outputX = MathUtil.clamp(outputX, -maxDriveRate, maxDriveRate);
        outputY = MathUtil.clamp(outputY, -maxDriveRate, maxDriveRate);
        outputTheta = MathUtil.clamp(outputTheta, -maxAngularRate, maxAngularRate);

        // Apply to Drivetrain
        // Note: FieldCentric request uses +X for forward and +Y for left
        drivetrain.setControl(this.drive
            .withVelocityX(outputX)
            .withVelocityY(outputY)
            .withRotationalRate(outputTheta));
    }

    @Override
    public boolean isFinished() {
        // Command finishes only when all three axes are within tolerance
        return controllerX.atSetpoint() && 
               controllerY.atSetpoint() && 
               controllerTheta.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        // Stop moving when finished or cancelled
        drivetrain.setControl(new SwerveRequest.Idle());
    }
}