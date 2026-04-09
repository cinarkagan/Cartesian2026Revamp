package frc.robot.subsystems.localization;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.PoseEstimate;
/*
 Limelight(1st) Piplines:
 * 0 = Localization
 * 
 * 
*/

public class LocalizationSubsystem extends SubsystemBase {
    LocalizerEstimate localizer;
    CommandSwerveDrivetrain swerveDrivetrain;
    public LocalizationSubsystem(CommandSwerveDrivetrain swerveDrivetrain) {
        this.localizer = new LocalizerEstimate(swerveDrivetrain);
        this.swerveDrivetrain = swerveDrivetrain;
    }
    
    @Override
    public void periodic() {
        /*
        PhotonTrackedTarget target = localizer.getBestofBothTarget();
        
        if (target.getPoseAmbiguity() < VisionConstants.ambiguityTol) { //If too high of an ambiguity, the update is rejected
            if (!target.equals(new PhotonTrackedTarget())) {
                Pose3d poseEstimate = localizer.getPoseEstimate(target);
                if (!poseEstimate.equals(new Pose3d())) {
                    swerveDrivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
                    swerveDrivetrain.addVisionMeasurement(
                        poseEstimate.toPose2d(),
                        1
                    );
                }
            }
        } */
        localizer.periodic();
    }
}
