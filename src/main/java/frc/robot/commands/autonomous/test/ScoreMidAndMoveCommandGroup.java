package frc.robot.commands.autonomous.test;


import com.sun.jdi.event.MonitorWaitedEvent;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.IntakeControlCommand;
import frc.robot.commands.SupersystemToPoseAutoCommand;
import frc.robot.commands.SupersystemToPoseCommand;
import frc.robot.commands.autonomous.TimerForwardAutoCommand;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.supersystems.ArmSupersystem;

public class ScoreMidAndMoveCommandGroup extends SequentialCommandGroup {
    public ScoreMidAndMoveCommandGroup(SwerveDrivetrain m_drive, ArmSupersystem m_super, WristSubsystem m_wrist) {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());
        super(
                new SupersystemToPoseAutoCommand(m_super, Constants.ArmSetpoints.MIDDLE_GOAL)
                        .alongWith(new WaitCommand(5).andThen(new IntakeControlCommand(m_wrist, -1.0)))
                        .raceWith(new WaitCommand(6)),
                new SupersystemToPoseAutoCommand(m_super, Constants.ArmSetpoints.STOW_POSITION)
                        .raceWith(new TimerForwardAutoCommand(m_drive, -0.5)),
                new InstantCommand(() -> {m_drive.resetGyro(180);})
        );
    }
}