// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.led.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.*;
import frc.robot.commands.autonomous.AutoFactory;
import frc.robot.commands.autonomous.Balance;
import frc.robot.subsystems.arm.ArmExtSubsystem;
import frc.robot.supersystems.ArmPose;
import frc.robot.supersystems.ArmSupersystem;
import lib.controllers.FootPedal;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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
    private CANdle m_candle;

    //Controllers
    private final CommandXboxController m_driveController = new CommandXboxController(Constants.DRIVER_PORT);
    private final CommandXboxController m_testController = new CommandXboxController(2);

    //Logged chooser for auto
    private final AutoFactory m_autoFactory;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        switch (Constants.CURRENT_MODE) {
            // Beta robot hardware implementation
            case HELIOS_V2:
                m_candle = new CANdle(22);
//                m_drive = new SwerveDrivetrain();
//                break;
            case HELIOS_V1:
                m_drive = new SwerveDrivetrain();
                m_wrist = new WristSubsystem();
                m_arm = new ArmAngleSubsystem();
                m_ext = new ArmExtSubsystem();
                m_super = new ArmSupersystem(m_arm, m_ext, m_wrist, m_drive);
                m_foot = new FootPedal(1);
                break;

            case SIM:
                break;

            // Default case, should be set to a replay mode
            default:
        }

        LiveWindow.disableAllTelemetry();

        m_autoFactory = new AutoFactory(m_super, m_drive, m_wrist, m_ext);

        
        // Configure the button bindings
        configureButtonBindings();
        configDashboard();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        if (m_candle != null) {
                if (DriverStation.getAlliance() == Alliance.Red) {
                    m_candle.animate(new ColorFlowAnimation(255, 0, 0));
                } else {
                    m_candle.animate(new ColorFlowAnimation(0, 0, 255));
                }
        }

        if (m_driveController != null) {
            m_drive.setDefaultCommand(new SwerveTeleopDrive(m_drive, m_driveController));
            m_arm.setDefaultCommand(new HoldArmAngleCommand(m_arm));


            m_driveController.button(7).onTrue(m_drive.resetGyroBase());

            m_driveController.leftTrigger().whileTrue(new IntakeControlCommand(m_wrist, -0.5));
            m_driveController.rightTrigger().whileTrue(new IntakeControlCommand(m_wrist, 1.0));

            if (Constants.CURRENT_MODE != Constants.Mode.HELIOS_V1) {
                m_driveController.x().whileTrue(
                        new SupersystemToPoseCommand(m_super, Constants.ArmSetpoints.INTAKE_BATTERY)
                                .alongWith(new IntakeControlCommand(m_wrist, 1.0, m_driveController.getHID())));
            }
            m_driveController.y().whileTrue(
                new SupersystemToPoseCommand(m_super, Constants.ArmSetpoints.INTAKE_BATTERY)
                        .alongWith(new IntakeControlCommand(m_wrist, 1.0, m_driveController.getHID())));

            m_driveController.a().whileTrue(new SupersystemToPoseCommand(m_super, Constants.ArmSetpoints.STOW_POSITION));
            m_driveController.b().whileTrue(
                    new SupersystemToPoseCommand(m_super, new ArmPose(0.0, () -> Constants.ArmSetpoints.HUMAN_HEIGHT.getValue(), () -> Constants.ArmSetpoints.HUMAN_WRIST.getValue()))
                            .alongWith(new IntakeControlCommand(m_wrist, 1.0, m_driveController.getHID())));

            //Auto Align with rumble for driving
            m_driveController.povLeft()
                    .whileTrue(m_drive.createPPSwerveController(SwerveDrivetrain.AlignmentOptions.LEFT_ALIGN));

            m_driveController.povUp()
                    .whileTrue(m_drive.createPPSwerveController(SwerveDrivetrain.AlignmentOptions.CENTER_ALIGN));

            m_driveController.povRight()
                    .whileTrue(m_drive.createPPSwerveController(SwerveDrivetrain.AlignmentOptions.RIGHT_ALIGN));

            m_driveController.povDown()
                    .whileTrue(new InstantCommand(() -> m_drive.resetPose(new Pose2d(10, 0, new Rotation2d()))));

            m_driveController.leftBumper().whileTrue(new SupersystemToPoseCommand(m_super, Constants.ArmSetpoints.HIGH_GOAL));
            m_driveController.rightBumper().whileTrue(new SupersystemToPoseCommand(m_super, Constants.ArmSetpoints.MIDDLE_GOAL));

            m_foot.leftPedal().whileTrue(m_drive.setSlowmodeFactory()).whileFalse(m_drive.setSlowmodeFactory());

            m_foot.middlePedal().onTrue(new PrintCommand("Middle Pedal Pressed"));
            m_foot.rightPedal().onTrue(new PrintCommand("Right Pedal Pressed"));
        } else {
            m_testController.a().whileTrue(m_ext.setArmSpeedFactory(0.25)).whileFalse(m_ext.setArmSpeedFactory(0.0));
            m_testController.b().whileTrue(m_ext.setArmSpeedFactory(-0.25)).whileTrue(m_ext.setArmSpeedFactory(0.0));
        }
    }

    /**
     * This method sets up Shuffleboard tabs for test commands
     */
    public void configDashboard() {
        ShuffleboardTab testCommands = Shuffleboard.getTab("Commands");

        testCommands.add("balance", new Balance(m_drive));

        testCommands.add("Toggle Angle Brake Mode", new ToggleArmBrakeModeCommand(m_arm)).withSize(2, 1);
        testCommands.add("Toggle Wrist Brake Mode", new InstantCommand(() -> m_wrist.toggleBrakeMode()).runsWhenDisabled());

        // Multiple stow positions for edge case testing
        testCommands.add("Test Stow Zone",
                new SupersystemToPoseCommand(m_super, new ArmPose(1, 30, 10))).withSize(2, 1);
        testCommands.add("Go To Stow",
                new SupersystemToPoseCommand(m_super, new ArmPose(0.0, 45, 0.0))).withSize(2, 1);
        testCommands.add("Test pose",
                new ArmPose(5, 90, 200)).withSize(2, 2);
        testCommands.add("Battery Intake",
                new SupersystemToPoseCommand(m_super, Constants.ArmSetpoints.INTAKE_BATTERY));

        // Arm Test Commands
        testCommands.add("Ground Intake Tipped Cone",
                new SupersystemToPoseCommand(m_super, new ArmPose(5.4, 325.0, 165.67))).withSize(2, 1);
        testCommands.add("Middle Score Zone",
                new SupersystemToPoseCommand(m_super, new ArmPose(0, 252.1, 99.7))).withSize(2, 1);
        testCommands.add("Cone Ground Intake",
                new SupersystemToPoseCommand(m_super, new ArmPose(0.0, 328, 177.5))).withSize(2, 1);
        testCommands.add("High Goal Setpoint",
                new SupersystemToPoseCommand(m_super, new ArmPose(23.3, 245, 86))).withSize(2, 1);
        testCommands.add("Human Player Station",
                new SupersystemToPoseCommand(m_super, new ArmPose(0, 236, 86))).withSize(2, 1);

        testCommands.add("Maintenance Mode",
                new MaitnanceModeCommandGroup(m_super).finallyDo((boolean isInterrupted) -> m_super.toggleAllBrakemode()));
        testCommands.add("Disable All Brakes",
                new InstantCommand(() -> m_super.disableAllBrakemode()).ignoringDisable(true));
        testCommands.add("Enable All Brakes",
                new InstantCommand(() -> m_super.enableAllBrakemode()).ignoringDisable(true));

        testCommands.add("Auto Balance", new AutoBalanceTransCommand(m_drive));
        testCommands.add("Reset Pose", new InstantCommand(() -> m_drive.resetPoseBase())).withSize(2, 1);

        testCommands.add("Align to Zero Degrees", m_drive.alignToAngle(0).asProxy());

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new InstantCommand(() ->
                m_drive.resetGyro(180))
                .andThen(m_autoFactory.getAutoRoutine());
    }

    public ArmSupersystem getArmSupersystem() {
        return m_super;
    }

}
