// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Swerve.SwerveDrivetrain;

public class AutoBalance extends CommandBase {
  private SwerveDrivetrain m_drive;

  private double currentAngle;
  private double error;
  private double drivePower;
  private int counter;
  private boolean isLevel;

  /** Creates a new AutoBalance. */
  public AutoBalance(SwerveDrivetrain drive) {
    m_drive = drive;
    currentAngle = 0;
    counter = 0;
    isLevel = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentAngle = m_drive.getGyroRoll().getDegrees();
    error = AutoConstants.kDesiredBalanceAngle - currentAngle;

    drivePower = Math.min(AutoConstants.kBalanceP * error, 1);
    // Limit max power
    if(Math.abs(drivePower) > 0.28){
      drivePower = Math.copySign(0.28, drivePower);
    }

    if(Math.abs(error) < 0.5 && counter == 10){
      isLevel = true;
    }
    if(Math.abs(error) < 0.5 && counter < 10){
      counter++;
    }

    m_drive.drive(drivePower, 0.0, 0.0);
    SmartDashboard.putNumber("Gyro Angle", currentAngle);
    SmartDashboard.putNumber("Drive Power", drivePower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isLevel; // If current error is within a threshold of one degree
  }
}