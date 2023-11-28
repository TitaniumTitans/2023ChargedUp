package frc.robot.subsystems.vision;


import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;


public class CameraSubsystem implements Subsystem {

    private final PhotonCamera m_camera;
    private final Transform3d robotToCam;
    private final CameraPose m_campose;

    private PhotonPoseEstimator m_photonPoseEstimator;

    private DriverStation.Alliance m_prevAlliance;
    private AprilTagFieldLayout m_aprilTagFieldLayout;

    private int m_prevTag = 1;
    private String m_name;

    /**
     * Creates a new CameraSubsystem
     *
     * @param camName Name of the camera in the system
     * @param camPose Transform3d of the camera's position relative to the robot
     */
    public CameraSubsystem(String camName, Transform3d camPose) {
        m_camera = new PhotonCamera(camName);
        m_prevAlliance = DriverStation.getAlliance();
        m_name = camName;

        SmartDashboard.putBoolean("Use Vision Filtering?", true);
        SmartDashboard.putBoolean(camName + "/filtering", true);

        robotToCam = camPose;

        // set origin point for allience. This is done because pathplanner flips paths for alliances weirdly,
        // causing need for this
        try {
            m_aprilTagFieldLayout =
                AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
           if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
               m_aprilTagFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
           } else {
               m_aprilTagFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
           }

        m_photonPoseEstimator = new PhotonPoseEstimator
            (m_aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE, m_camera,
                robotToCam);
        } catch (IOException e) {
            throw new IllegalStateException(e);
        }

        m_campose = new CameraPose(camName, robotToCam.getTranslation(), robotToCam.getRotation());
        SmartDashboard.putBoolean(camName + "/update original", false);
    }

    /**
     * Gets optional estimatedRobotPose from AprilTag data
     * @param prevEstimatedRobotPose Pose 2d reference position for the Photon Vision pose estimator to work off of
     * @return Optional estimatedRobotPose, a Pose3d and a timestamp in seconds
     */
    public Optional<EstimatedRobotPose> getPose(Pose2d prevEstimatedRobotPose) {
        m_photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);

        // Check for alllience switch, used mainly for non-comp testing
        if (DriverStation.getAlliance() != m_prevAlliance) {
            m_prevAlliance = DriverStation.getAlliance();
            if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
                m_aprilTagFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);

            } else {
                m_aprilTagFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
            }
            m_photonPoseEstimator.setFieldTags(m_aprilTagFieldLayout);
        }

        PhotonPipelineResult camResult = m_camera.getLatestResult();

        if (!SmartDashboard.getBoolean(m_name + "/filtering", true)) {
            return m_photonPoseEstimator.update(camResult);
        } else {
            ArrayList<PhotonTrackedTarget> goodTargets = new ArrayList<>();

            double robotToTagDist;

            if (!camResult.hasTargets()) {
                return Optional.empty();
            } else {
                for (PhotonTrackedTarget target : camResult.getTargets()) {
                    /**
                     * Get the current target position and check if it is a valid target.
                     */
                    Optional<Pose3d> targetPose = m_aprilTagFieldLayout.getTagPose(target.getFiducialId());

                    if (targetPose.isEmpty()) {
                        continue;
                    }

                    if (SmartDashboard.getBoolean("Use Vision Filtering?", true)) {

                        /**
                         * Calculate the position of the robot based on the best target result and alternate target result.
                         */
                        Pose3d bestTagPose = targetPose.get().transformBy(target.getBestCameraToTarget().inverse())
                                .transformBy(robotToCam);
                        Pose3d altTagPose = targetPose.get().transformBy(target.getAlternateCameraToTarget().inverse())
                                .transformBy(robotToCam);

                        SmartDashboard.putNumber("Best Tag X", bestTagPose.getX());
                        SmartDashboard.putNumber("Best Tag Y", bestTagPose.getY());
                        SmartDashboard.putNumber("Alt Tag X", altTagPose.getX());
                        SmartDashboard.putNumber("Alt Tag Y", altTagPose.getY());

                        /**
                         * Calculate the distance between the bestTagPose and the previous pose.
                         * And calculate the distance between the altTagPose and the previous pose.
                         */
                        double bestPoseAndPrevPoseDist = Math.sqrt((bestTagPose.getX() - prevEstimatedRobotPose.getX()
                                * (bestTagPose.getX() - prevEstimatedRobotPose.getX()))
                                + ((bestTagPose.getY() - prevEstimatedRobotPose.getY())
                                * (bestTagPose.getY() - prevEstimatedRobotPose.getY())));
                        double altPoseAndPrevPoseDist = Math.sqrt((altTagPose.getX() - prevEstimatedRobotPose.getX()
                                * (altTagPose.getX() - prevEstimatedRobotPose.getX()))
                                + ((altTagPose.getY() - prevEstimatedRobotPose.getY())
                                * (altTagPose.getY() - prevEstimatedRobotPose.getY())));

                        /**
                         * check which robot position based on the two different results is closer to the previous pose.
                         * Then calculate the distance between the desired robot position and the tag position.
                         */
                        if (bestPoseAndPrevPoseDist < altPoseAndPrevPoseDist) {
                        /*
                        robotToTagDist = Math.sqrt(((bestTagPose.getX() - targetPose.get().getX())
                                * (bestTagPose.getX() - targetPose.get().getX()))
                                + ((bestTagPose.getY() - targetPose.get().getY())
                                * (bestTagPose.getY() - targetPose.get().getY())));

                         */

                            robotToTagDist = bestTagPose.minus(targetPose.get()).getTranslation().getNorm();
                        } else {
                        /*
                        robotToTagDist = Math.sqrt(((altTagPose.getX() - targetPose.get().getX())
                                * (altTagPose.getX() - targetPose.get().getX()))
                                + ((altTagPose.getY() - targetPose.get().getY())
                                * (altTagPose.getY() - targetPose.get().getY())));
                        */
                            robotToTagDist = altTagPose.minus(targetPose.get()).getTranslation().getNorm();
                        }

                        SmartDashboard.putNumber("Robot to Tag Distance", robotToTagDist);

                        /**
                         * Check if the ambiguity of the tag is below the predetermined threshold.
                         * And check if the distance between the robot and tag is below the predetermined threshold.
                         */
                        if (target.getPoseAmbiguity() <= Constants.DriveConstants.CAM_AMBIGUITY_THRESHOLD.getValue()
                                && robotToTagDist < Constants.DriveConstants.CAM_DISTANCE_THRESHOLD.getValue()) {
                            goodTargets.add(target);
                        }
                    } else {
                        goodTargets = (ArrayList<PhotonTrackedTarget>) camResult.getTargets();
                    }
                }


                /**
                 * Create a new photon pipeline with the list of good targets
                 */
                PhotonPipelineResult filteredCamResults = new PhotonPipelineResult
                        (camResult.getLatencyMillis(), goodTargets);
                filteredCamResults.setTimestampSeconds(camResult.getTimestampSeconds());

                return m_photonPoseEstimator.update(filteredCamResults);
            }
        }
    }


    public Pose2d getTagPose() {
        PhotonPipelineResult results = m_camera.getLatestResult();
        Optional<Pose3d> pose;

        if (results.hasTargets()) {
            pose = m_aprilTagFieldLayout.getTagPose(results.getBestTarget().getFiducialId());
            m_prevTag = results.getBestTarget().getFiducialId();
        } else {
            pose = m_aprilTagFieldLayout.getTagPose(m_prevTag);
        }

        // return the pose of the tag currently visible to the camera
        if(pose.isPresent()) {
            return pose.get().toPose2d();
        } else {
            return new Pose2d();
        }
    }

    public Transform3d robotToTag() {
        PhotonPipelineResult result = m_camera.getLatestResult();

        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            return target.getBestCameraToTarget().plus(robotToCam.inverse());
        } else {
            return new Transform3d();
        }
    }

    public int getTagID() {
        PhotonPipelineResult results = m_camera.getLatestResult();

        if (results.hasTargets()) {
            m_prevTag = results.getBestTarget().getFiducialId();
            return results.getBestTarget().getFiducialId();
        } else {
            return m_prevTag;
        }
    }

    public void useSingleTag(int tagId) {
        AprilTagFieldLayout field = new AprilTagFieldLayout(
                List.of(new AprilTag(tagId, new Pose3d())),
                0.0,
                0.0
        );

        m_photonPoseEstimator.setFieldTags(field);
    }

    public void resetTagField() {
        m_photonPoseEstimator.setFieldTags(m_aprilTagFieldLayout);
    }

    @Override
    public void periodic() {
        if (m_campose.hasChanged()) {
            m_photonPoseEstimator = new PhotonPoseEstimator(m_aprilTagFieldLayout,
                    PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
                    m_camera,
                    new Transform3d(m_campose.getTranslation(), m_campose.getRotation()));
        }

        if (SmartDashboard.getBoolean(m_name + "/update original", false)) {
            m_campose.setOriginal();
            SmartDashboard.putBoolean(m_name + "/update original", false);
        }
        m_campose.updateVisualization();
    }
}
