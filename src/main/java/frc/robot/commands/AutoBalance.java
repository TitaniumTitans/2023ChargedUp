// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class AutoBalance extends CommandBase {
  private SwerveDrivetrain m_drive;

  private double m_currentAngle;
  private double m_error;
  private double m_drivePower;
  private int m_counter;
  private boolean m_isLevel;

  /** Creates a new AutoBalance. */
  public AutoBalance(SwerveDrivetrain drive) {
    m_drive = drive;
    m_currentAngle = 0;
    m_counter = 0;
    m_isLevel = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_currentAngle = m_drive.getGyroRoll().getDegrees();
    m_error = AutoConstants.DESIRED_BALANCE_ANGLE - m_currentAngle;

    m_drivePower = Math.min(AutoConstants.BALANCE_P * m_error, 1);
    // Limit max power
    if (Math.abs(m_drivePower) > 0.28) {
      m_drivePower = Math.copySign(0.28, m_drivePower);
    }

    if (Math.abs(m_error) < 0.5 && m_counter == 10) {
      m_isLevel = true;
    }
    if (Math.abs(m_error) < 0.5 && m_counter < 10) {
      m_counter++;
    }

    m_drive.setModuleStates(m_drivePower, 0.0, 0.0);
    SmartDashboard.putNumber("Gyro Angle", m_currentAngle);
    SmartDashboard.putNumber("Drive Power", m_drivePower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isLevel; // If current error is within a threshold of one degree
  }
}