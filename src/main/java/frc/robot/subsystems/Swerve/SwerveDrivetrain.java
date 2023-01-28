// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import java.lang.reflect.Field;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Vision.CameraSubsystem;

public class SwerveDrivetrain extends SubsystemBase {
  private SwerveIO m_io;
  private SwerveDriveOdometry m_odometry;
  private SwerveIOInputsAutoLogged inputs;
  private Field2d m_field;

  private boolean fieldRelative;

  private CameraSubsystem frontPVCam;
  private String FRONT_CAM_NAME = "FrontPiCam";
  private Transform3d FRONT_CAM_POSE = new Transform3d
    (new Translation3d(0.5, 0.0, 0.5), new Rotation3d());
  private Pose2d m_prevPose;
  private Field2d m_PVField;

  /** Creates a new SwerveDrivetrain. */
  public SwerveDrivetrain(SwerveIO io) {
    m_io = io;
    m_odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, 
      m_io.getGyroYaw(), 
      m_io.getModulePositions());

    inputs = new SwerveIOInputsAutoLogged();

    m_field = new Field2d();
    SmartDashboard.putData("Field", m_field);
    
    fieldRelative = false;

    // CAMERA CONFIG
    frontPVCam = new CameraSubsystem(FRONT_CAM_NAME, FRONT_CAM_POSE);
    m_prevPose = new Pose2d();
    m_PVField = new Field2d();
  }

  public Rotation2d getGyroYaw(){
    return m_io.getGyroYaw();
  }

  public Rotation2d getGyroRoll(){
    return m_io.getGyroRoll();
  }

  public SwerveModulePosition[] getModulePostitions(){
    return m_io.getModulePositions();
  }

  public SwerveModuleState[] getModuleStates(){
    return m_io.getModuleStates();
  }

  public void drive(double xTranslation, double yTranslation, double zRotation){
    m_io.setModuleStates(xTranslation, yTranslation, zRotation, fieldRelative);
  }

  public void setModuleStates(SwerveModuleState[] states){
    m_io.setModuleStates(states);
  }

  public void setAbsoluteAngles(){
    m_io.setAbsoluteAngles();
  }

  public void resetGyro(){
    m_io.resetGyro();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(m_io.getGyroYaw(), m_io.getModulePositions());

    m_io.updateInputs(inputs);
    Logger.getInstance().processInputs("Drive", inputs);
    m_field.setRobotPose(m_odometry.getPoseMeters());

    SmartDashboard.putBoolean("Field Relative", fieldRelative);
    SmartDashboard.putNumber("Gyro", getGyroYaw().getDegrees());

    // CAMERA:
    SmartDashboard.putData("PV field", m_PVField);
    Optional<EstimatedRobotPose> frontEPose = frontPVCam.getPose(m_prevPose);
    SmartDashboard.putBoolean("Cam pose present", frontEPose.isPresent());
    if (frontEPose.isPresent())
    {
      EstimatedRobotPose frontPose = frontEPose.get();
      m_PVField.setRobotPose(frontPose.estimatedPose.toPose2d());
      m_prevPose = frontPose.estimatedPose.toPose2d();
    }
  }

  public CommandBase resetGyroBase(){
    return runOnce(() -> {resetGyro();});
  }

  public CommandBase toggleFieldRelative(){
    return runOnce(() -> {fieldRelative = !fieldRelative;});
  }

  public Pose2d getPose(){
    return m_odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose){
    m_odometry.resetPosition(getGyroYaw(), getModulePostitions(), pose);
  }

  public void resetPose(){
    resetPose(new Pose2d());
  }
}
