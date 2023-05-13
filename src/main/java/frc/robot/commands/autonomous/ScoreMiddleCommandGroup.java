package frc.robot.commands.autonomous;


import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.IntakeControlCommand;
import frc.robot.commands.SupersystemToPoseAutoCommand;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.supersystems.ArmSupersystem;

public class ScoreMiddleCommandGroup extends SequentialCommandGroup {
    public ScoreMiddleCommandGroup(ArmSupersystem m_super, WristSubsystem wrist) {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());

        super(new SupersystemToPoseAutoCommand(m_super, Constants.ArmSetpoints.MIDDLE_GOAL),
                new PrintCommand("Stopped first command"),
                new IntakeControlCommand(wrist, -1.0).raceWith(
                        new WaitCommand(0.5)
                ),
                new IntakeControlCommand(wrist, 0.0).raceWith(
                        new WaitCommand(0.0)
                ),
                new SupersystemToPoseAutoCommand(m_super, Constants.ArmSetpoints.STOW_POSITION));
    }
}