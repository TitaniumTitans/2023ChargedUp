// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.SwerveTeleopDrive;
import frc.robot.commands.Autonomous.AutoFactory;
import frc.robot.subsystems.Swerve.SwerveDrivetrain;
import frc.robot.subsystems.Swerve.SwerveFalconIO;
import frc.robot.subsystems.Swerve.SwerveNeoIO;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  //Subsystems
  private SwerveDrivetrain m_drive; 

  //Controllers
  private final CommandXboxController m_driveController = new CommandXboxController(Constants.driverPort);

  //Logged chooser for auto
  private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Modes");
  private final AutoFactory m_AutoFactory;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
  switch (Constants.currentMode){
    // Beta robot hardware implementation
    case THANOS:
      m_drive = new SwerveDrivetrain(new SwerveNeoIO());
      m_AutoFactory = new AutoFactory(m_drive);
      break;
    
    case ALPHA:
      m_drive = new SwerveDrivetrain(new SwerveFalconIO());
      m_AutoFactory = new AutoFactory(m_drive);
      break;

    case SIM:
      m_drive = new SwerveDrivetrain(new SwerveNeoIO());
      m_AutoFactory = new AutoFactory(m_drive);
      break;

    // Default case, should be set to a replay mode
    default:
      m_drive = new SwerveDrivetrain(new SwerveFalconIO());
      m_AutoFactory = new AutoFactory(m_drive);
  }
    // Configure the button bindings
    configureButtonBindings();
    configAutoChooser();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_drive.setDefaultCommand(new SwerveTeleopDrive(m_drive, m_driveController));

    m_driveController.button(7).onTrue(m_drive.resetGyroBase());
    m_driveController.button(8).onTrue(m_drive.toggleFieldRelative());
  }

  /**
   * Use this method to add autonomous routines to a sendable chooser
   */
  public void configAutoChooser(){
    autoChooser.addDefaultOption("Default Trajectory", AutoFactory.getDefaultTrajectory(m_drive));
    autoChooser.addOption("Event Map Trajectory", AutoFactory.getDefaultPathWithEvents(m_drive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_AutoFactory.getAuto();
  }
}
