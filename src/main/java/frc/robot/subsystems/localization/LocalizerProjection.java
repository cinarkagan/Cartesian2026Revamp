package frc.robot.subsystems.localization;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.VisionConstants;

public class LocalizerProjection {
    PhotonCamera cam1;
    PhotonCamera cam2;

    public LocalizerProjection() {
        cam1 = new PhotonCamera(VisionConstants.camera1Name);
        cam2 = new PhotonCamera(VisionConstants.camera2Name);
    }

    public List<PhotonPipelineResult> getLatestResult(PhotonCamera cam) {
        return cam.getAllUnreadResults();
    }

    public PhotonTrackedTarget getBestTarget(PhotonCamera cam) {
        List<PhotonPipelineResult> latestResult = getLatestResult(cam);
        PhotonTrackedTarget bestOfBestTarget = new PhotonTrackedTarget();
        for (int i = 0; i < latestResult.size(); i++) {
            PhotonPipelineResult result = latestResult.get(i);
            
            if (result.hasTargets()) {
                List<PhotonTrackedTarget> targets = result.getTargets();
                PhotonTrackedTarget bestTarget = result.getTargets().get(0);
                for (int s = 0; s<targets.size(); s++) {
                    PhotonTrackedTarget target = targets.get(s);
                    if (s==0) {bestTarget = target;}
                    else if (bestTarget != null) {
                        if (bestTarget.getPoseAmbiguity() > target.getPoseAmbiguity()) {
                            bestTarget = target;
                        }
                    }
                }
                if (bestOfBestTarget.equals(new PhotonTrackedTarget())) {
                    if (bestOfBestTarget.getPoseAmbiguity() > bestTarget.poseAmbiguity) {
                        bestOfBestTarget = bestTarget;
                    }
                } else {
                    bestOfBestTarget = bestTarget;
                }
            }
        }
        return bestOfBestTarget;
    }
    public PhotonTrackedTarget getBestofBothTarget() {
        PhotonTrackedTarget cam1Target = getBestTarget(cam1);
        PhotonTrackedTarget cam2Target = getBestTarget(cam2);

        if (cam1Target.equals(new PhotonTrackedTarget()) && cam2Target.equals(new PhotonTrackedTarget())) {
            return new PhotonTrackedTarget();
        } else if ((cam1Target.equals(new PhotonTrackedTarget()))&& (!cam2Target.equals(new PhotonTrackedTarget()))) {
            return cam2Target;
        } else if ((!cam1Target.equals(new PhotonTrackedTarget()))&& (cam2Target.equals(new PhotonTrackedTarget()))) {
            return cam1Target;
        } else {
            if (cam1Target.getPoseAmbiguity() > cam2Target.getPoseAmbiguity()) {
                return cam2Target;
            } else {
                return cam1Target;
            }
        }
    }
    //TODO: Which camera has the best, according to that give the kRobotCam pose thing
    public Pose3d getPoseReprojection(PhotonTrackedTarget target) {
        if (VisionConstants.layout.getTagPose(target.getFiducialId()).isPresent()) {
            Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), VisionConstants.layout.getTagPose(target.getFiducialId()).get(), VisionConstants.kRobotToCam1);
            SmartDashboard.putString("Vision/RobotPose", robotPose.toString());
            SmartDashboard.putNumber("Vision/RobotPoseX", robotPose.getX());
            SmartDashboard.putNumber("Vision/RobotPoseY", robotPose.getY());
            SmartDashboard.putNumber("Vision/RobotPoseHeading", robotPose.getRotation().getAngle());
            return robotPose;
        }
        return new Pose3d();
    }
}
