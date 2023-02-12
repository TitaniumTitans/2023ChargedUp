// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
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

import java.util.Optional;

public class SwerveDrivetrain extends SubsystemBase {
  private SwerveIO m_io;
  private SwerveDriveOdometry m_odometry;
  private SwerveDrivePoseEstimator m_poseEstimator;
  private SwerveIOInputsAutoLogged inputs;
  private Field2d m_field;

  private boolean fieldRelative;

   private CameraSubsystem m_frontPVCam;
  private String FRONT_CAM_NAME = "FrontPiCam";
  private Transform3d FRONT_CAM_POSE = new Transform3d
    (new Translation3d(Units.inchesToMeters(11.4), 
    Units.inchesToMeters(-3.75), Units.inchesToMeters(15.5)), new Rotation3d(
      0.0, 0.0, 0.0
    ));
  private Pose2d m_prevPose;

  private int m_counter = 0;
  private int m_loopCycles = 100;
  private double m_allowAbleAngleError = 3.0;

  /** Creates a new SwerveDrivetrain. */
  public SwerveDrivetrain(SwerveIO io) {
    m_io = io;
    // m_odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, 
    //   m_io.getGyroYaw(), 
    //   m_io.getModulePositions());

     m_poseEstimator = new SwerveDrivePoseEstimator
       (Constants.DriveConstants.DRIVE_KINEMATICS, m_io.getGyroYaw(),
       m_io.getModulePositions(), new Pose2d());

    inputs = new SwerveIOInputsAutoLogged();

    m_field = new Field2d();
    // SmartDashboard.putData("Field", m_field);
    
    fieldRelative = false;

    // CAMERA CONFIG
     m_frontPVCam = new CameraSubsystem(FRONT_CAM_NAME, FRONT_CAM_POSE);
    m_prevPose = new Pose2d();
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
    // This method will be called once per scheduler run
    // updatePoseEstimator();
    // SmartDashboard.putNumber
    //   ("Estimated Pose X", m_poseEstimator.getEstimatedPosition().getX());
    // SmartDashboard.putNumber
    //   ("Estimated Pose Y", m_poseEstimator.getEstimatedPosition().getY());
    // // SmartDashboard.putData(m_field);

    // m_poseEstimator.update(getGyroRoll(), getModulePostitions());
//
//    m_io.updateInputs(inputs);
//    Logger.getInstance().processInputs("Drive", inputs);
//    // m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());
//    SmartDashboard.putData("Field", m_field);
//
//    SmartDashboard.putBoolean("Field Relative", fieldRelative);
//    SmartDashboard.putNumber("Gyro", getGyroYaw().getDegrees());
//
//    Rotation2d[] array = m_io.getCancoder();
//
//    SmartDashboard.putNumber("FL Mod CANCoder", array[0].getDegrees());
//    SmartDashboard.putNumber("FR Mod CANCoder", array[1].getDegrees());
//    SmartDashboard.putNumber("BL Mod CANCoder", array[2].getDegrees());
//    SmartDashboard.putNumber("BR Mod CANCoder", array[3].getDegrees());

    // resets the module positions to absolute after so many loops cycles in enable
    // TODO Remove comment when actually testing
    // if (m_counter == m_loopCycles) {
    //   m_io.setAbsoluteAngles();
    //   m_counter++;
    // } else {
    //   m_counter++;
    // }

    // periodically checks the angle of the CANCoders and compares it to the angles
    // of the azimuth
    //TODO remove comment when actually testing
    // if (m_counter == m_loopCycles) {
    //   if (Math.abs(array[0].getDegrees() - inputs.flAngleDeg) > m_allowAbleAngleError) {
    //     m_io.setIndevidualAngle(1);
    //   }

    //   if (Math.abs(array[1].getDegrees() - inputs.frAngleDeg) > m_allowAbleAngleError) {
    //     m_io.setIndevidualAngle(2);
    //   }

    //   if (Math.abs(array[2].getDegrees() - inputs.blAngleDeg) > m_allowAbleAngleError) {
    //     m_io.setIndevidualAngle(3);
    //   }

    //   if (Math.abs(array[3].getDegrees() - inputs.brAngleDeg) > m_allowAbleAngleError) {
    //     m_io.setIndevidualAngle(4);
    //   }

    //   m_counter = 0;
    // } else {
    //   m_counter++;
    // }


//     CAMERA:
     Optional<EstimatedRobotPose> frontEPose =
             m_frontPVCam.getPose(m_poseEstimator.getEstimatedPosition());
     SmartDashboard.putBoolean("Cam pose present", frontEPose.isPresent());
     if (frontEPose.isPresent())
     {
       EstimatedRobotPose frontPose = frontEPose.get();
       m_prevPose = frontPose.estimatedPose.toPose2d();
     }

    updatePoseEstimator();
    SmartDashboard.putNumber("Pose estimator pose x", m_poseEstimator.getEstimatedPosition().getX());
    SmartDashboard.putNumber("Pose estimator pose y", m_poseEstimator.getEstimatedPosition().getY());

    SmartDashboard.putData("PE field", m_field);
  }

   public void updatePoseEstimator() {
     m_poseEstimator.update(m_io.getGyroYaw(), m_io.getModulePositions());

     Optional<EstimatedRobotPose> estimateCamPose =
       m_frontPVCam.getPose(m_prevPose);
     SmartDashboard.putBoolean("POSE ESTIMATOR isPresent", estimateCamPose.isPresent());
     if (estimateCamPose.isPresent())
     {
       EstimatedRobotPose camPose = estimateCamPose.get();
       m_poseEstimator.addVisionMeasurement
         (camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
     }
     m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());
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
