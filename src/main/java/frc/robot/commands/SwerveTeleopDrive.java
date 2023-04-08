// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import lib.utils.Utils;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** An example command that uses an example subsystem. */
public class SwerveTeleopDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveDrivetrain m_drive;
  private final CommandXboxController m_driverController;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SwerveTeleopDrive(SwerveDrivetrain subsystem, CommandXboxController controller) {
    m_drive = subsystem;
    m_driverController = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = -m_driverController.getLeftY();
    double y = -m_driverController.getLeftX();
    double z = -m_driverController.getRightX() / 1.8;

    if (m_drive.getSlowmode()) {
      x *= 0.5;
      y *= 0.5;
      z *= 0.5;
    }

    x = Utils.deadBand(x);
    y = Utils.deadBand(y);
    z = Utils.deadBand(z);

    x = x * (Constants.CURRENT_MODE == Constants.Mode.HELIOS_V1 ? Constants.ModuleConstants.MAX_SPEED_L2_MPS : Constants.ModuleConstants.MAX_SPEED_L3_MPS);
    y = y * (Constants.CURRENT_MODE == Constants.Mode.HELIOS_V1 ? Constants.ModuleConstants.MAX_SPEED_L2_MPS : Constants.ModuleConstants.MAX_SPEED_L3_MPS);
    z = z * (Constants.CURRENT_MODE == Constants.Mode.HELIOS_V1 ? Constants.ModuleConstants.MAX_SPEED_L2_MPS : Constants.ModuleConstants.MAX_SPEED_L3_MPS);
    m_drive.drive(x, y, z);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
