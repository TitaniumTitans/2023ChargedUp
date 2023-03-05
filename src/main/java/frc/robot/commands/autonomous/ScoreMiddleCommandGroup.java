package frc.robot.commands.autonomous;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ArmToScoreCommand;
import frc.robot.commands.SupersystemToPoseCommand;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.supersystems.ArmSupersystem;

public class ScoreMiddleCommandGroup extends SequentialCommandGroup {
    public ScoreMiddleCommandGroup(ArmSupersystem m_super, WristSubsystem wrist) {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());
        super(new SupersystemToPoseCommand(m_super, Constants.ArmSetpoints.MIDDLE_GOAL_NON_STOW),
                wrist.setIntakeSpeedFactory(-1.00),
                wrist.setIntakeSpeedFactory(0.0),
                new SupersystemToPoseCommand(m_super, Constants.ArmSetpoints.STOW_POSITION));
    }
}