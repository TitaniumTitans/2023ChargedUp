// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.SupersystemToPoseCommand;
import frc.robot.commands.ToggleArmBrakeModeCommand;
import frc.robot.subsystems.arm.ArmExtSubsystem;
import frc.robot.supersystems.ArmPose;
import frc.robot.supersystems.ArmSupersystem;
import lib.controllers.FootPedal;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.commands.SwerveTeleopDrive;
import frc.robot.commands.autonomous.AutoUtils;
import frc.robot.subsystems.arm.ArmAngleSubsystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.wrist.WristSubsystem;
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
  private ArmAngleSubsystem m_arm;
  private ArmExtSubsystem m_ext;
  private ArmSupersystem m_super;
  private FootPedal m_foot;

  //Controllers
  private final CommandXboxController m_driveController = new CommandXboxController(Constants.DRIVER_PORT);

  //Logged chooser for auto
  private final LoggedDashboardChooser<Command> m_autoChooser = new LoggedDashboardChooser<>("Auto Modes");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.CURRENT_MODE) {
      // Beta robot hardware implementation
      case THANOS:
      case HELIOS:
        m_drive = new SwerveDrivetrain();
        m_wrist = new WristSubsystem();
        m_arm = new ArmAngleSubsystem();
        m_ext = new ArmExtSubsystem();
        m_super = new ArmSupersystem(m_arm, m_ext, m_wrist);
        m_foot = new FootPedal(1);
        break;

      case SIM:
        break;

      // Default case, should be set to a replay mode
      default:
    }
    LiveWindow.disableAllTelemetry();

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
//    m_arm.setDefaultCommand(m_arm.setAngleSpeedFactory(0.0));
//    m_wrist.setDefaultCommand(m_wrist.setWristPowerFactory(0.0));
//    m_ext.setDefaultCommand(m_ext.setArmSpeedFactory(0.0));

    m_driveController.button(7).onTrue(m_drive.resetGyroBase());
    m_driveController.start().onTrue(m_drive.toggleFieldRelative());

    m_driveController.leftTrigger().whileTrue(m_wrist.setIntakeSpeedFactory(-1.0))
                    .whileFalse(m_wrist.setIntakeSpeedFactory(0.0));
    m_driveController.rightTrigger().whileTrue(m_wrist.setIntakeSpeedFactory(1.0))
            .whileFalse(m_wrist.setIntakeSpeedFactory(0.0));

    m_driveController.b().whileTrue(m_wrist.setWristPowerFactory(0.20))
      .whileFalse(m_wrist.setWristPowerFactory(0.0));
    m_driveController.a().whileTrue(m_wrist.setWristPowerFactory(-0.20))
      .whileFalse(m_wrist.setWristPowerFactory(0.0));


    m_driveController.x().whileTrue(m_arm.setAngleSpeedFactory(0.5))
      .whileFalse(m_arm.setAngleSpeedFactory(0.0));
    m_driveController.y().whileTrue(m_arm.setAngleSpeedFactory(-0.5))
            .whileFalse(m_arm.setAngleSpeedFactory(0.0));

      m_driveController.rightBumper().whileTrue(m_ext.setArmSpeedFactory(0.5))
              .whileFalse(m_ext.setArmSpeedFactory(0.0));
    m_driveController.leftBumper().whileTrue(m_ext.setArmSpeedFactory(-0.5))
            .whileFalse(m_ext.setArmSpeedFactory(0.0));

    m_foot.leftPedal().onTrue(new PrintCommand("Left Pedal Pressed!"));
    m_foot.middlePedal().onTrue(new PrintCommand("Middle Pedal Pressed"));
    m_foot.rightPedal().onTrue(new PrintCommand("Right Pedal Pressed"));
  }

  /**
   * Use this method to add autonomous routines to a sendable chooser
   */
  public void configAutoChooser() {
    m_autoChooser.addDefaultOption("Default Trajectory", AutoUtils.getDefaultTrajectory(m_drive));
    m_autoChooser.addOption("Event Map Trajectory", AutoUtils.getPathWithEvents(m_drive));
  }

  /**
   * This method sets up Shuffleboard tabs for test commands
   */
  public void configDashboard() {
    ShuffleboardTab testCommands = Shuffleboard.getTab("Commands");

    testCommands.add("Toggle Angle Brake Mode", new ToggleArmBrakeModeCommand(m_arm)).withSize(2, 1);

    testCommands.add("Test Stow Zone", new SupersystemToPoseCommand(m_super, new ArmPose(1, 30, 10))).withSize(2, 1);
    testCommands.add("Go To Stow", new SupersystemToPoseCommand(m_super, new ArmPose(0.0, 45, 0.0))).withSize(2, 1);
    //TODO: Find out why angle 200 crashes in Piecewise interval
    testCommands.add("Test pose", new ArmPose(5, 90, 200)).withSize(2, 2);

    testCommands.add("Reset Pose", new InstantCommand(() -> m_drive.resetPoseBase())).withSize(2, 1);

    testCommands.add("Ground Intake Tipped Cone", new SupersystemToPoseCommand(m_super, new ArmPose(5.4, 325.0, 165.67))).withSize(2, 1);
    testCommands.add("Middle Score Zone", new SupersystemToPoseCommand(m_super, new ArmPose(0, 252.1, 99.7))).withSize(2, 1);
    testCommands.add("Cone Ground Intake", new SupersystemToPoseCommand(m_super, new ArmPose(0.0, 328, 177.5)));
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
