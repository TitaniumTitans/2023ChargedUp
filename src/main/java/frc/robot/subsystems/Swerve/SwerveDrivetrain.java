// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveDrivetrain extends SubsystemBase {
  private SwerveIO m_io;
  private SwerveDriveOdometry m_odometry;
  private SwerveIOInputsAutoLogged inputs;

  private boolean fieldRelative;
  /** Creates a new SwerveDrivetrain. */
  public SwerveDrivetrain(SwerveIO io) {
    m_io = io;
    m_odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, 
      m_io.getGyro(), 
      m_io.getModulePositions());

    inputs = new SwerveIOInputsAutoLogged();
    
    fieldRelative = false;
  }

  public Rotation2d getGyro(){
    return m_io.getGyro();
  }

  public SwerveModulePosition[] getModulePostitions(){
    return m_io.getModulePositions();
  }

  public SwerveModuleState[] getModuleStates(){
    return m_io.getModuleStates();
  }

  public void setModuleStates(double xTranslation, double yTranslation, double zRotation){
    m_io.setModuleStates(xTranslation, yTranslation, zRotation, fieldRelative);
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
    m_odometry.update(m_io.getGyro(), m_io.getModulePositions());

    m_io.updateInputs(inputs);
    Logger.getInstance().processInputs("Drive", inputs);
  }

  public CommandBase resetGyroBase(){
    return runOnce(() -> {resetGyro();});
  }

  public CommandBase toggleFieldRelative(){
    return runOnce(() -> {fieldRelative = !fieldRelative;});
  }
}
