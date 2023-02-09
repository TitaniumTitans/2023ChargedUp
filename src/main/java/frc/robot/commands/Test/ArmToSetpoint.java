// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm.ArmSubsystem;

public class ArmToSetpoint extends CommandBase {
  private double armSetpoint;
  private ArmSubsystem arm;

  /** Creates a new ArmToSetpoint. */
  public ArmToSetpoint(ArmSubsystem arm, double setpoint) {
    this.arm = arm;
    armSetpoint = setpoint;

    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(arm.armAngle > ArmConstants.kForwardLimit || arm.armAngle < ArmConstants.kReverseLimit){}
    else {
      arm.armAngle = armSetpoint;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
