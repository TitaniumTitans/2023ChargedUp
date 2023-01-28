// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.ArmSubSystem;

public class MoveArmAngle extends CommandBase {
  private ArmSubSystem m_arm;
  private double speed;
  /** Creates a new MoveArmAngle. */
  public MoveArmAngle(ArmSubSystem arm, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.speed = speed;
    m_arm = arm;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setAngleSpeed(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_arm.getArmAngle() > m_arm.kForwardLimitDegrees && speed > 0){
      m_arm.setAngleSpeed(0.0);
    }
    else if(m_arm.getArmAngle() < m_arm.kReverseLimitDegrees && speed < 0){
      m_arm.setAngleSpeed(0.0);
    }
    else{
      m_arm.setAngleSpeed(speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.setAngleSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
