// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.IntakeControlCommand;
import frc.robot.commands.SupersystemToPoseCommand;
import frc.robot.commands.ToggleArmBrakeModeCommand;
import frc.robot.commands.autonomous.TimerForwardAutoCommand;
import frc.robot.commands.autonomous.test.ScoreMidAndMoveCommandGroup;
import frc.robot.commands.autonomous.test.TestAutoWithArm;
import frc.robot.commands.test.TestModuleCommand;
import frc.robot.commands.test.TestSwerveCommand;
import frc.robot.commands.test.TestSwerveRotationCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.arm.ArmExtSubsystem;
import frc.robot.supersystems.ArmPose;
import frc.robot.supersystems.ArmSupersystem;
import lib.controllers.FootPedal;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.commands.autonomous.AutoUtils;
import frc.robot.subsystems.arm.ArmAngleSubsystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.wrist.WristSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.util.HashMap;

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

  private static SwerveAutoBuilder defaultAutoBuilder;

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
      defaultAutoBuilder = new SwerveAutoBuilder(
              m_drive::getPose,
              m_drive::resetPose,
              Constants.DriveConstants.DRIVE_KINEMATICS,
              new PIDConstants(1.5, 0, 0),
              new PIDConstants(1, 0, 0),
              m_drive::setModuleStates,
              new HashMap<>(),
              true,
              m_drive
      );

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

    m_driveController.leftTrigger().whileTrue(new IntakeControlCommand(m_wrist, -1.0));
    m_driveController.rightTrigger().whileTrue(new IntakeControlCommand(m_wrist, 1.0));

    m_driveController.x().whileTrue(
            new SupersystemToPoseCommand(m_super, Constants.ArmSetpoints.INTAKE_CUBE)
            .alongWith(new IntakeControlCommand(m_wrist, 1.0)));
    m_driveController.y().whileTrue(
            new SupersystemToPoseCommand(m_super, Constants.ArmSetpoints.INTAKE_CONE)
            .alongWith(new IntakeControlCommand(m_wrist, 1.0)));

    m_driveController.a().whileTrue(new SupersystemToPoseCommand(m_super, Constants.ArmSetpoints.STOW_POSITION));
    m_driveController.b().whileTrue(
            new SupersystemToPoseCommand(m_super, Constants.ArmSetpoints.HUMAN_PLAYER_STATION)
            .alongWith(new IntakeControlCommand(m_wrist, 1.0)));

    m_driveController.leftBumper().whileTrue(new SupersystemToPoseCommand(m_super, Constants.ArmSetpoints.HIGH_GOAL));
    m_driveController.rightBumper().whileTrue(new SupersystemToPoseCommand(m_super, Constants.ArmSetpoints.MIDDLE_GOAL_NON_STOW));
//    m_driveController.a()
//            .and(m_driveController.b())
//            .and(m_driveController.x())
//            .and(m_driveController.y())
//            .onFalse(new SupersystemToPoseCommand(m_super, Constants.ArmSetpoints.STOW_POSITION));

    m_foot.leftPedal().whileTrue(m_drive.setSlowmodeFactory()).whileFalse(m_drive.setSlowmodeFactory());

    m_foot.middlePedal().onTrue(new PrintCommand("Middle Pedal Pressed"));
    m_foot.rightPedal().onTrue(new PrintCommand("Right Pedal Pressed"));
  }

  /**
   * Use this method to add autonomous routines to a sendable chooser
   */
  public void configAutoChooser() {
    m_autoChooser.addDefaultOption("Drive Backwards", new TimerForwardAutoCommand(m_drive, 0.5));
    m_autoChooser.addOption("Score Mid Move", new ScoreMidAndMoveCommandGroup(m_drive, m_super, m_wrist));
  }

  /**
   * This method sets up Shuffleboard tabs for test commands
   */
  public void configDashboard() {
    ShuffleboardTab testCommands = Shuffleboard.getTab("Commands");
    ShuffleboardTab testTrajectories = Shuffleboard.getTab("Trajectories");

    testCommands.add("Toggle Angle Brake Mode", new ToggleArmBrakeModeCommand(m_arm)).withSize(2, 1);
    // Multiple stow positions for edge case testing
    testCommands.add("Test Stow Zone", new SupersystemToPoseCommand(m_super, new ArmPose(1, 30, 10))).withSize(2, 1);
    testCommands.add("Go To Stow", new SupersystemToPoseCommand(m_super, new ArmPose(0.0, 45, 0.0))).withSize(2, 1);
    testCommands.add("Test pose", new ArmPose(5, 90, 200)).withSize(2, 2);


    // Arm Test Commands
    testCommands.add("Ground Intake Tipped Cone", new SupersystemToPoseCommand(m_super, new ArmPose(5.4, 325.0, 165.67))).withSize(2, 1);
    testCommands.add("Middle Score Zone", new SupersystemToPoseCommand(m_super, new ArmPose(0, 252.1, 99.7))).withSize(2, 1);
    testCommands.add("Cone Ground Intake", new SupersystemToPoseCommand(m_super, new ArmPose(0.0, 328, 177.5))).withSize(2, 1);
    testCommands.add("High Goal Setpoint", new SupersystemToPoseCommand(m_super, new ArmPose(23.3, 245, 86))).withSize(2, 1);
    testCommands.add("Human Player Station", new SupersystemToPoseCommand(m_super, new ArmPose(0, 236, 86))).withSize(2, 1);

    testCommands.add("Vert Stow", new SupersystemToPoseCommand(m_super, Constants.ArmSetpoints.VERT_STOW_POSE));

    // Swerve Test Commands
//    testCommands.add("Swerve Forward", new TestSwerveCommand(m_drive, 0));
//    testCommands.add("Swerve Right", new TestSwerveCommand(m_drive, 90));
//    testCommands.add("Swerve Backwards", new TestSwerveCommand(m_drive, 180));
//    testCommands.add("Swerve Left", new TestSwerveCommand(m_drive, 270));

//    testCommands.add("Swerve Clockwise", new TestSwerveRotationCommand(m_drive, false));
//    testCommands.add("Swerve CounterClockwise", new TestSwerveRotationCommand(m_drive, true));

//    testCommands.add("FL Module Test", new TestModuleCommand(m_drive, 0));
//    testCommands.add("BL Module Test", new TestModuleCommand(m_drive, 2));

    testCommands.add("Auto Balance", new AutoBalance(m_drive));
    testCommands.add("Reset Pose", new InstantCommand(() -> m_drive.resetPoseBase())).withSize(2, 1);

//    testTrajectories.add("Move With Arm", TestAutoWithArm.getAuto(m_drive, m_super));
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
