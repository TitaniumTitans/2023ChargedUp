package frc.robot.commands.autonomous;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.IntakeControlCommand;
import frc.robot.commands.SupersystemToPoseAutoCommand;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.supersystems.ArmSupersystem;

public class ScoreOneOnlyCommandGroup extends SequentialCommandGroup {
    public ScoreOneOnlyCommandGroup(ArmSupersystem m_super, WristSubsystem m_wrist,
                                    AutoUtils.ScoringHeights height) {
        addCommands(m_super.runIntake(0.0));
        if (height == AutoUtils.ScoringHeights.HIGH)
        {
            addCommands(new SupersystemToPoseAutoCommand(m_super, Constants.ArmSetpoints.HIGH_GOAL));
        }
        else
        {
            addCommands(new SupersystemToPoseAutoCommand(m_super, Constants.ArmSetpoints.MIDDLE_GOAL));
        }

        addCommands(new IntakeControlCommand(m_wrist, -1.0).raceWith(new WaitCommand(0.5)));
        addCommands(new IntakeControlCommand(m_wrist, 0.0).raceWith(new WaitCommand(0.0)));
        addCommands(new SupersystemToPoseAutoCommand(m_super, Constants.ArmSetpoints.STOW_POSITION));
    }
}