// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.math.trajectory.Trajectory;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveDrivetrain extends SubsystemBase {
  private SwerveIO m_io;
  private SwerveDriveOdometry m_odometry;
  private SwerveIOInputsAutoLogged inputs;

  private Field2d m_field;


  private boolean fieldRelative;
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
    m_field = new Field2d();
    SmartDashboard.putData("Field", m_field);
  }

  public Rotation2d getGyroYaw(){
    return m_io.getGyroYaw();
  }

  public SwerveModulePosition[] getModulePostitions(){
    return m_io.getModulePositions();
  }

  public SwerveModuleState[] getModuleStates(){
    return m_io.getModuleStates();
  }

  public void drive(double xTranslation, double yTranslation, double zRotation){
    m_io.drive(xTranslation, yTranslation, zRotation, fieldRelative);
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

  public void setFieldTrajectory(Trajectory traj){
    m_field.getObject("traj").setTrajectory(traj); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(m_io.getGyroYaw(), m_io.getModulePositions());

    m_io.updateInputs(inputs);
    m_field.setRobotPose(m_odometry.getPoseMeters());

    Logger.getInstance().processInputs("Drive", inputs);
    m_field.setRobotPose(m_odometry.getPoseMeters());

    SmartDashboard.putBoolean("Field Relative", fieldRelative);
    SmartDashboard.putNumber("Gyro", getGyroYaw().getDegrees());


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
