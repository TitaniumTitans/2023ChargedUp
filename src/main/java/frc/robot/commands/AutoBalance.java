// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class AutoBalance extends CommandBase {
  private final SwerveDrivetrain m_drive;

  private final Timer m_timer;
  private final Timer m_balanceTimer;

  private final PIDController m_balanceController;

  private boolean m_isLevel;

  /** Creates a new AutoBalance. */
  public AutoBalance(SwerveDrivetrain drive) {
    m_drive = drive;
    m_timer = new Timer();
    m_balanceTimer = new Timer();
    m_balanceController = new PIDController(AutoConstants.BALANCE_P, 0.0, AutoConstants.BALANCE_D);
    m_isLevel = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_balanceController.setSetpoint(AutoConstants.DESIRED_BALANCE_ANGLE);
    m_isLevel = false;

    m_timer.reset();
    m_timer.stop();

    m_balanceTimer.reset();
    m_balanceTimer.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Gets how far the robot is tilted and calculates proper drive power from it
    double m_currentAngle = m_drive.getGyroPitch().getDegrees();
    double m_error = AutoConstants.DESIRED_BALANCE_ANGLE - m_currentAngle;
    double m_drivePower = Math.min(m_balanceController.calculate(m_currentAngle), 1);

    // Limit max power
    m_drivePower = MathUtil.clamp(m_drivePower, -0.5, 0.5);

    double rollRateBalance = m_drive.getGyroPitchRate();
    // if robot is tilting forward, stop moving so we don't overshoot the balance
    if (Math.abs(rollRateBalance) >= 0.3) {
      m_balanceTimer.start();
      m_drivePower = Math.copySign(0.15, -m_drive.getGyroPitch().getDegrees());
    }

    if (m_balanceTimer.hasElapsed(0.01)) {
//      m_drivePower = Math.copySign(0.15, -m_drive.getGyroPitch().getDegrees());
      m_drivePower = 0.0;
    }
    if (m_balanceTimer.hasElapsed(0.75)) {
      m_balanceTimer.stop();
      m_balanceTimer.reset();
    }

    // Counter for checking if robot is truly balanced
    if (Math.abs(m_error) < AutoConstants.ACCEPTABLE_BALANCE_ANGLE && !m_timer.hasElapsed(0.1)) {
      // if robot is balanced, stop moving and start a timer
      m_timer.start();
      m_drivePower = 0.0;
      SmartDashboard.putBoolean("Timer Started", true);
    }
    if (Math.abs(m_error) < AutoConstants.ACCEPTABLE_BALANCE_ANGLE && m_timer.hasElapsed(2)) {
      // If robot has been balanced for 10 seconds, stop the command
      m_timer.stop();
      m_isLevel = true;
      SmartDashboard.putBoolean("Timer Started", false);
    }
    if (Math.abs(m_error) < AutoConstants.ACCEPTABLE_BALANCE_ANGLE && m_timer.get() < 2) {
      m_drivePower = 0.0;
    }
    if (m_timer.hasElapsed(0) && Math.abs(m_error) > AutoConstants.ACCEPTABLE_BALANCE_ANGLE) {
      // if robot is no longer balanced, stop and reset the timer
      m_timer.stop();
      m_timer.reset();
      SmartDashboard.putBoolean("Timer Started", false);
    }

    SmartDashboard.putNumber("Timer Count", m_timer.get());
    SmartDashboard.putBoolean("Command should stop", m_isLevel);
    m_drive.drive(m_drivePower, 0.0, 0.0);

    // Logging values for debugging
    SmartDashboard.putNumber("Gyro Error", m_error);
//    SmartDashboard.putNumber("Drive Power", m_drivePower);
//    SmartDashboard.putNumber("Roll Rate Balance", rollRateBalance);
//    SmartDashboard.putNumber("Roll Rate Balance Raw", m_drive.getGyroRollPitch());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Set the module azimuths to 45 degrees so robot doesn't slip off the charge station
    m_drive.drive(0.0, 0.0, 0.1);
    m_drive.drive(0.0, 0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isLevel; // If current error is within a threshold of one degree
  }
}