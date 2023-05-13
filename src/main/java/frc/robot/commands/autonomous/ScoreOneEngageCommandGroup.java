package frc.robot.commands.autonomous;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.IntakeControlCommand;
import frc.robot.commands.SupersystemToPoseAutoCommand;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.supersystems.ArmSupersystem;

public class ScoreOneEngageCommandGroup extends SequentialCommandGroup {
    public ScoreOneEngageCommandGroup(SwerveDrivetrain m_swerve, ArmSupersystem m_super, WristSubsystem m_wrist, AutoUtils.StartingZones start, AutoUtils.ScoringHeights height) {
        addCommands(m_super.runIntake(0.0));
        if (height == AutoUtils.ScoringHeights.HIGH) {
            addCommands(new SupersystemToPoseAutoCommand(m_super, Constants.ArmSetpoints.HIGH_GOAL));
        } else {
            addCommands(new SupersystemToPoseAutoCommand(m_super, Constants.ArmSetpoints.MIDDLE_GOAL));
        }

        addCommands(new IntakeControlCommand(m_wrist, -0.3)
                .raceWith(new WaitCommand(0.25)));
        addCommands(new IntakeControlCommand(m_wrist, 0.0)
                .raceWith(new WaitCommand(0.0)));

        addCommands(new SupersystemToPoseAutoCommand(m_super, Constants.ArmSetpoints.STOW_POSITION));
        addCommands(new BalanceCommandGroup(m_swerve, start));
    }
}