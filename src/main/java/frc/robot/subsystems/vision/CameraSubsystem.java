package frc.robot.subsystems.vision;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;


public class CameraSubsystem implements Subsystem {

    private final PhotonCamera m_camera;

    private PhotonPoseEstimator m_photonPoseEstimator;

    /**
     * Creates a new CameraSubsystem
     * @param camName Name of the camera in the system
     * @param camPose Transform3d of the camera's position relative to the robot
     */
    public CameraSubsystem(String camName, Transform3d camPose) {
        m_camera = new PhotonCamera(camName);

        try {
            AprilTagFieldLayout aprilTagFieldLayout = 
                AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
            m_photonPoseEstimator = new PhotonPoseEstimator
                (aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE, m_camera, 
                    camPose);
        } catch (IOException e) {
            throw new IllegalStateException(e);
        }
    }

    /**
     * Gets optional estimatedRobotPose from AprilTag data
     * @param prevEstimatedRobotPose Pose 2d reference position for the Photon Vision pose estimator to work off of
     * @return Optional estimatedRobotPose, a Pose3d and a timestamp in seconds
     */
    public Optional<EstimatedRobotPose> getPose(Pose2d prevEstimatedRobotPose) {
        m_photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);

        PhotonPipelineResult camResult = m_camera.getLatestResult();
        ArrayList<PhotonTrackedTarget> goodTargets = new ArrayList<>();

        if(!camResult.hasTargets()) {
            return Optional.empty();
        } else {
            for (PhotonTrackedTarget target: camResult.getTargets()) {
                if (target.getPoseAmbiguity() < Constants.DriveConstants.CAM_AMBIGUITY_THRESHOLD.getValue()) {
                    goodTargets.add(target);
                }
            }
        }

        PhotonPipelineResult filteredCamResult =
                new PhotonPipelineResult(camResult.getLatencyMillis(), goodTargets);
        filteredCamResult.setTimestampSeconds(camResult.getTimestampSeconds());

        return m_photonPoseEstimator.update(filteredCamResult);
    }




}
