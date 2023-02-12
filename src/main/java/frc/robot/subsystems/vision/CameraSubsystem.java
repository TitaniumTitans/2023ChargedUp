package frc.robot.subsystems.vision;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import java.io.IOException;
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
            throw new RuntimeException(e);
        }
    }

    /**
     * Gets optional estimatedRobotPose from AprilTag data
     * @param prevEstimatedRobotPose Pose 2d refrence position for the Photon Vision pose estimator to work off of
     * @return Optional estimatedRobotPose, a Pose3d and a timestamp in seconds
     */
    public Optional<EstimatedRobotPose> getPose(Pose2d prevEstimatedRobotPose) {
        m_photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);

        Optional<EstimatedRobotPose> estimate = m_photonPoseEstimator.update();
        if (estimate.isPresent()) {
            EstimatedRobotPose pose = estimate.get();
            SmartDashboard.putNumber("Vision only X pose", pose.estimatedPose.getX());
            SmartDashboard.putNumber("vision only Y pose", pose.estimatedPose.getY());
        }

        return estimate;
    }




}
