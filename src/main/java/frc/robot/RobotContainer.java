// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Test.TestArmFullCommandGroup;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.Test.WristToSetpointCommand;
import frc.robot.commands.Test.ArmExtendToSetpoint;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.commands.SwerveTeleopDrive;
import frc.robot.commands.Autonomous.AutoUtils;
import frc.robot.commands.Test.ArmToSetpoint;
import frc.robot.subsystems.Arm.ArmIONeo;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.swerve.SwerveNeoIO;
import frc.robot.subsystems.Wrist.WristIONeo;
import frc.robot.subsystems.Wrist.WristSubsystem;
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
  private WristSubsystem m_wrist;
  private SwerveDrivetrain m_drive; 
  private ArmSubsystem m_arm;

  //Controllers
  private final CommandXboxController m_driveController = new CommandXboxController(Constants.DRIVER_PORT);

  //Logged chooser for auto
  private final LoggedDashboardChooser<Command> m_autoChooser = new LoggedDashboardChooser<>("Auto Modes");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
  switch (Constants.CURRENT_MODE) {
    // Beta robot hardware implementation
    case THANOS:
      m_drive = new SwerveDrivetrain(new SwerveNeoIO());
      m_wrist = new WristSubsystem(new WristIONeo());
      m_arm = new ArmSubsystem(new ArmIONeo());
      break;
    
    case HELIOS:
      m_drive = new SwerveDrivetrain(new SwerveNeoIO());
      m_wrist = new WristSubsystem(new WristIONeo());
      m_arm = new ArmSubsystem(new ArmIONeo());
      break;

    case SIM:
      break;

    // Default case, should be set to a replay mode
    default:
  }
    // Configure the button bindings
    configureButtonBindings();
    configAutoChooser();
    configDashboard();
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
    m_driveController.start().onTrue(m_drive.toggleFieldRelative());

//    m_driveController.a().whileTrue(new TestArmFullCommandGroup(3, 180, -20, m_arm, m_wrist));
//    m_driveController.b().whileTrue(new TestArmFullCommandGroup(7, 90, 20, m_arm, m_wrist));

    m_driveController.leftTrigger().whileTrue(m_wrist.setIntakeSpeedFactory(-1.0))
                    .whileFalse(m_wrist.setIntakeSpeedFactory(0.0));
    m_driveController.rightTrigger().whileTrue(m_wrist.setIntakeSpeedFactory(1.0))
            .whileFalse(m_wrist.setIntakeSpeedFactory(0.0));
//
    m_driveController.x().whileTrue(m_wrist.setWristPowerFactory(0.15))
      .whileFalse(m_wrist.setWristPowerFactory(0.0));
    m_driveController.y().whileTrue(m_wrist.setWristPowerFactory(-0.15))
      .whileFalse(m_wrist.setWristPowerFactory(0.0));


    m_driveController.a().whileTrue(m_arm.setArmAngleSpeedFactory(0.5))
      .whileFalse(m_arm.setArmAngleSpeedFactory(0.0));
    m_driveController.b().whileTrue(m_arm.setArmAngleSpeedFactory(-0.5))
            .whileFalse(m_arm.setArmAngleSpeedFactory(0.0));

    m_driveController.povDown().whileTrue(new TestArmFullCommandGroup
            (4.4, 62.7, -36.1, m_arm, m_wrist));
    m_driveController.povUp().whileTrue((new TestArmFullCommandGroup
            (8, 142.0, 0, m_arm, m_wrist)));
    m_driveController.povRight().whileTrue(new TestArmFullCommandGroup(
            4.00, 70, 0, m_arm, m_wrist
    ));
    m_driveController.povLeft().whileTrue(new TestArmFullCommandGroup(
            25, 133.5, -3.3, m_arm, m_wrist
    ));

//    m_driveController.rightBumper().whileTrue(new WristToSetpointCommand(m_wrist, 45.0));
//    m_driveController.leftBumper().whileTrue(new WristToSetpointCommand(m_wrist, 0.0));

      m_driveController.rightBumper().whileTrue(m_arm.setArmExtentionSpeedFactory(0.5))
              .whileFalse(m_arm.setArmExtentionSpeedFactory(0.0));
    m_driveController.leftBumper().whileTrue(m_arm.setArmExtentionSpeedFactory(-0.5))
            .whileFalse(m_arm.setArmExtentionSpeedFactory(0.0));
  }

  /**
   * Use this method to add autonomous routines to a sendable chooser
   */
  public void configAutoChooser(){
    m_autoChooser.addDefaultOption("Default Trajectory", AutoUtils.getDefaultTrajectory(m_drive));
    m_autoChooser.addOption("Event Map Trajectory", AutoUtils.getPathWithEvents(m_drive));
  }

  /**
   * This method sets up Shuffleboard tabs for test commands
   */
  public void configDashboard(){
    ShuffleboardTab testCommands = Shuffleboard.getTab("Commands");

    testCommands.add("Arm to 90", new ArmToSetpoint(m_arm, 90));
    testCommands.add("Arm to 40", new ArmToSetpoint(m_arm, 40));
    testCommands.add("Arm to 140", new ArmToSetpoint(m_arm, 140));

    testCommands.add("Reset Pose", new InstantCommand(() -> m_drive.resetPose()));

    testCommands.add("Wrist to 0", new WristToSetpointCommand(m_wrist, 0));
    testCommands.add("Wrist to 20", new WristToSetpointCommand(m_wrist, 20));
    testCommands.add("Wrist to -20", new WristToSetpointCommand(m_wrist, -20));

    testCommands.add("Arm extend to 3", new ArmExtendToSetpoint(m_arm, 3));
    testCommands.add("Arm extend to 5", new ArmExtendToSetpoint(m_arm, 5));
    testCommands.add("Arm extend to 7", new ArmExtendToSetpoint(m_arm, 7));

    testCommands.add("Test for full arm",
            new TestArmFullCommandGroup(3, 180, -20, m_arm, m_wrist));
    testCommands.add("Test for full arm 2.0",
            new TestArmFullCommandGroup(7, 90, 20, m_arm, m_wrist));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoChooser.get();
  }
}
