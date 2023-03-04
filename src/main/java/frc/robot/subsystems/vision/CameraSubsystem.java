package frc.robot.subsystems.vision;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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

    private AprilTagFieldLayout m_aprilTagFieldLayout;

    /**
     * Creates a new CameraSubsystem
     * @param camName Name of the camera in the system
     * @param camPose Transform3d of the camera's position relative to the robot
     */
    public CameraSubsystem(String camName, Transform3d camPose) {
        m_camera = new PhotonCamera(camName);

        try {
            m_aprilTagFieldLayout =
                AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
            m_photonPoseEstimator = new PhotonPoseEstimator
                (m_aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE, m_camera,
                    camPose);
        } catch (IOException e) {
            throw new IllegalStateException(e);
        }
    }

    /**
     * Gets optional estimatedRobotPose from AprilTag data
     * @param prevEstimatedRobotPose Pose 2d reference position for the Photon Vision pose estimator to work off of
     * @param robotToCam 3d transform offset of the camera to the robot
     * @return Optional estimatedRobotPose, a 3d pose and a timestamp in seconds
     */
    public Optional<EstimatedRobotPose> getPose(Pose2d prevEstimatedRobotPose, Transform3d robotToCam) {
        m_photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);

        PhotonPipelineResult camResult = m_camera.getLatestResult();

        ArrayList<PhotonTrackedTarget> goodTargets = new ArrayList<>();

        Pose3d filteredPose;

        if(!camResult.hasTargets()) {
            return Optional.empty();
        } else {
            for (PhotonTrackedTarget target: camResult.getTargets())
            {
                /**
                 * Probably don't need this.
                 */
//                Optional<Pose3d> targetPose = m_aprilTagFieldLayout.getTagPose(target.getFiducialId());
//
//                Pose3d bestTagPose = targetPose.get().transformBy(target.getBestCameraToTarget().inverse())
//                        .transformBy(robotToCam);
//                Pose3d altTagPose = targetPose.get().transformBy(target.getAlternateCameraToTarget().inverse())
//                        .transformBy(robotToCam);
//                double bestPoseAndPrevPoseDist = Math.sqrt((bestTagPose.getX() - prevEstimatedRobotPose.getX()
//                * (bestTagPose.getX() - prevEstimatedRobotPose.getX()))
//                + ((bestTagPose.getY() - prevEstimatedRobotPose.getY())
//                * (bestTagPose.getY() - prevEstimatedRobotPose.getY())));
//                double altPoseAndPrevPoseDist = Math.sqrt((altTagPose.getX() - prevEstimatedRobotPose.getX()
//                        * (altTagPose.getX() - prevEstimatedRobotPose.getX()))
//                        + ((altTagPose.getY() - prevEstimatedRobotPose.getY())
//                        * (altTagPose.getY() - prevEstimatedRobotPose.getY())));
//                if (bestPoseAndPrevPoseDist < altPoseAndPrevPoseDist) {
//                    filteredPose = bestTagPose;
//                } else {
//                    filteredPose = altTagPose;
//                }
                if (target.getPoseAmbiguity() <= Constants.DriveConstants.CAM_AMBIGUITY_THRESHOLD.getValue())
                {
                    goodTargets.add(target);
                }
            }
        }


        PhotonPipelineResult filteredCamResults = new PhotonPipelineResult
                (camResult.getLatencyMillis(), goodTargets);
        filteredCamResults.setTimestampSeconds(camResult.getTimestampSeconds());

        return m_photonPoseEstimator.update(filteredCamResults);
    }




}
