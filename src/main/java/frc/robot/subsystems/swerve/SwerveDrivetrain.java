// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants;
import frc.robot.subsystems.Vision.CameraSubsystem;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.EstimatedRobotPose;
import frc.robot.Constants.DriveConstants;

import java.util.Optional;

public class SwerveDrivetrain extends SubsystemBase {
  private SwerveIO m_io;
  private SwerveDriveOdometry m_odometry;
  private SwerveDrivePoseEstimator m_poseEstimator;
  private SwerveIOInputsAutoLogged inputs;
  private Field2d m_field;
  private Field2d m_testField;

  private boolean fieldRelative;

   private CameraSubsystem m_frontPVCam;
  private String FRONT_CAM_NAME = "FrontPiCam";
  private Transform3d FRONT_CAM_POSE = new Transform3d
    (new Translation3d(Units.inchesToMeters(11.4), 
    Units.inchesToMeters(-3.75), Units.inchesToMeters(15.5)), new Rotation3d(
      0.0, 0.0, 0.0
    ));
  private Pose2d m_prevPose;

  int count = 0;

//  private SwerveDriveOdometry m_odometry;

  /** Creates a new SwerveDrivetrain. */
  public SwerveDrivetrain(SwerveIO io) {
    m_io = io;
     m_odometry = new SwerveDriveOdometry(DriveConstants.DRIVE_KINEMATICS,
       m_io.getGyroYaw(),
       m_io.getModulePositions());

     m_poseEstimator = new SwerveDrivePoseEstimator
       (Constants.DriveConstants.DRIVE_KINEMATICS, m_io.getGyroYaw(),
       m_io.getModulePositions(), new Pose2d());

    inputs = new SwerveIOInputsAutoLogged();

    m_field = new Field2d();
    // SmartDashboard.putData("Field", m_field);

    m_testField = new Field2d();
    
    fieldRelative = false;

    // CAMERA CONFIG
     m_frontPVCam = new CameraSubsystem(FRONT_CAM_NAME, FRONT_CAM_POSE);
    m_prevPose = new Pose2d();

    m_io.setAbsoluteAngles();
  }

  public Rotation2d getGyroYaw() {
    return m_io.getGyroYaw();
  }

  public Rotation2d getGyroRoll() {
    return m_io.getGyroRoll();
  }

  public SwerveModulePosition[] getModulePostitions() {
    return m_io.getModulePositions();
  }

  public SwerveModuleState[] getModuleStates() {
    return m_io.getModuleStates();
  }

  public void drive(double xTranslation, double yTranslation, double zRotation) {
    m_io.setModuleStates(xTranslation, yTranslation, zRotation, fieldRelative);
  }

  public void setModuleStates(SwerveModuleState[] states) {
    m_io.setModuleStates(states);
  }

  public void setAbsoluteAngles() {
    m_io.setAbsoluteAngles();
  }

  public void resetGyro() {
    m_io.resetGyro();
  }

  @Override
  public void periodic() {
//     This method will be called once per scheduler run
//     SmartDashboard.putNumber
//       ("Estimated Pose X", m_poseEstimator.getEstimatedPosition().getX());
//     SmartDashboard.putNumber
//       ("Estimated Pose Y", m_poseEstimator.getEstimatedPosition().getY());

    m_io.updateInputs(inputs);
    Logger.getInstance().processInputs("Drive", inputs);
    updatePoseEstimator();
     m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());
    SmartDashboard.putData("Field", m_field);
//
    SmartDashboard.putBoolean("Field Relative", fieldRelative);
    SmartDashboard.putNumber("Gyro", getGyroYaw().getDegrees());

//     CAMERA:
     Optional<EstimatedRobotPose> frontEPose =
             m_frontPVCam.getPose(m_poseEstimator.getEstimatedPosition());
     SmartDashboard.putBoolean("Cam pose present", frontEPose.isPresent());
     if (frontEPose.isPresent())
     {
       EstimatedRobotPose frontPose = frontEPose.get();
       m_prevPose = frontPose.estimatedPose.toPose2d();
     }

    SmartDashboard.putNumber("Pose estimator pose x", m_poseEstimator.getEstimatedPosition().getX());
    SmartDashboard.putNumber("Pose estimator pose y", m_poseEstimator.getEstimatedPosition().getY());

//    SmartDashboard.putData("PE field", m_field);

    double[] modAngles = m_io.getAngles();
    SmartDashboard.putNumberArray("Module Positions", modAngles);

    m_odometry.update(m_io.getGyroYaw(), m_io.getModulePositions());
    m_testField.setRobotPose(m_odometry.getPoseMeters());
    SmartDashboard.putData("test field", m_testField);

    Rotation2d[] cancoderAngles = m_io.getCancoderAngles();

    for (int i = 0; i < 4; i++) {
      SmartDashboard.putNumber("Module " + i, cancoderAngles[i].getDegrees());
    }


  }

   public void updatePoseEstimator() {
     var dummyPosition =  m_io.getModulePositions();

     int i = 0;

     for (var pos : dummyPosition) {
//       SmartDashboard.putNumber("Mod Pos Rot" + i, pos.angle.getDegrees());
//       SmartDashboard.putNumber("Mod Pos Speed" + i, pos.distanceMeters);
//       SmartDashboard.putNumber("Mod Pos Sin" + i, pos.angle.getSin());
//       SmartDashboard.putNumber("Mod Pos Cos" + i, pos.angle.getCos());
//       pos.angle = new Rotation2d();
       i++;
     }

     m_poseEstimator.update(m_io.getGyroYaw(), dummyPosition);

     Optional<EstimatedRobotPose> estimateCamPose =
       m_frontPVCam.getPose(m_prevPose);
     SmartDashboard.putBoolean("POSE ESTIMATOR isPresent", estimateCamPose.isPresent());
     if (estimateCamPose.isPresent())
     {
       EstimatedRobotPose camPose = estimateCamPose.get();
       m_poseEstimator.addVisionMeasurement
         (camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
     }
//     m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());
   }

  public CommandBase resetGyroBase() {
    return runOnce(() -> {resetGyro();});
  }

  public CommandBase toggleFieldRelative() {
    return runOnce(() -> {fieldRelative = !fieldRelative;});
  }

  public Pose2d getPose() {
    return new Pose2d();
    // return m_poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose) {
     m_poseEstimator.resetPosition(getGyroYaw(), getModulePostitions(), pose);
  }

  public void resetPose() {
    resetPose(new Pose2d());
  }
}
