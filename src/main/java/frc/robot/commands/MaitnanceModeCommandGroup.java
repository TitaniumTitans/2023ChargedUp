package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.supersystems.ArmSupersystem;

public class MaitnanceModeCommandGroup extends SequentialCommandGroup {
    public MaitnanceModeCommandGroup(ArmSupersystem m_super) {
        addCommands(new SupersystemToPoseAutoCommand(m_super, Constants.ArmSetpoints.VERT_STOW_POSE));
        addCommands(new InstantCommand(() -> m_super.toggleAllBrakemode()));
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}