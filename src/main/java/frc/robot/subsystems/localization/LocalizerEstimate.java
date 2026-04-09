package frc.robot.subsystems.localization;


import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class LocalizerEstimate {
    private final PhotonCamera cam1;
    private final PhotonCamera cam2;
    private final PhotonPoseEstimator photonEstimator1;
    private final PhotonPoseEstimator photonEstimator2;

    CommandSwerveDrivetrain commandSwerveDrivetrain;
    public LocalizerEstimate(CommandSwerveDrivetrain commandSwerveDrivetrain) {
        cam1 = new PhotonCamera(VisionConstants.camera1Name);
        cam2 = new PhotonCamera(VisionConstants.camera2Name);
        photonEstimator1 = new PhotonPoseEstimator(VisionConstants.layout, VisionConstants.kRobotToCam1);
        photonEstimator2 = new PhotonPoseEstimator(VisionConstants.layout, VisionConstants.kRobotToCam2);
        this.commandSwerveDrivetrain = commandSwerveDrivetrain;
    }

    public void periodic() {
        List<PhotonPipelineResult> cam1Result = cam1.getAllUnreadResults();

        Optional<EstimatedRobotPose> visionEst1 = Optional.empty();
        for (var result : cam1Result) { 
            List<PhotonTrackedTarget> filteredTargets = new ArrayList<>();
            for (var target : result.getTargets()) {
                if (target.getPoseAmbiguity() < VisionConstants.ambiguityTol) {
                    filteredTargets.add(target);
                }
            }

            PhotonPipelineResult filteredResult = new PhotonPipelineResult(
                result.metadata,
                filteredTargets,
                Optional.empty()
            );
            if (filteredResult.hasTargets()) {
                visionEst1 = photonEstimator1.estimateCoprocMultiTagPose(filteredResult);
                if (visionEst1.isEmpty()) {
                    visionEst1 = photonEstimator1.estimateLowestAmbiguityPose(filteredResult);
                }
                
                visionEst1.ifPresent(est -> {
                    commandSwerveDrivetrain.addVisionMeasurement(
                        est.estimatedPose.toPose2d(), 
                        est.timestampSeconds
                    );
                });
            }
        }
        List<PhotonPipelineResult> cam2Result = cam2.getAllUnreadResults();

        Optional<EstimatedRobotPose> visionEst2 = Optional.empty();
        for (var result : cam2Result) { 
            List<PhotonTrackedTarget> filteredTargets = new ArrayList<>();
            for (var target : result.getTargets()) {
                if (target.getPoseAmbiguity() < VisionConstants.ambiguityTol) {
                    filteredTargets.add(target);
                }
            }

            PhotonPipelineResult filteredResult = new PhotonPipelineResult(
                result.metadata,
                filteredTargets,
                Optional.empty()
            );
            if (filteredResult.hasTargets()) {

                visionEst2 = photonEstimator2.estimateCoprocMultiTagPose(filteredResult);
                if (visionEst2.isEmpty()) {
                    visionEst2 = photonEstimator2.estimateLowestAmbiguityPose(filteredResult);
                }
                
                visionEst2.ifPresent(est -> {
                    commandSwerveDrivetrain.addVisionMeasurement(
                        est.estimatedPose.toPose2d(), 
                        est.timestampSeconds
                    );
                });
            }
        }
    }
}