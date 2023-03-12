package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.wrist.WristSubsystem;


public class IntakeControlCommand extends CommandBase {

    WristSubsystem m_wrist;

    double m_speed;

    XboxController m_controller;

    public IntakeControlCommand(WristSubsystem wrist, double speed, XboxController controller) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        m_wrist = wrist;
        m_speed = speed;
        m_controller = controller;
        addRequirements();
    }

    public IntakeControlCommand(WristSubsystem wrist, double speed) {
        this(wrist, speed, null);
    }

    @Override
    public void execute() {
        m_wrist.setIntakeSpeed(m_speed);
        if(m_controller != null && m_wrist.isStalling()) {
            m_controller.setRumble(GenericHID.RumbleType.kBothRumble, 1);
        }
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_wrist.setIntakeSpeed(0.0);
        m_controller.setRumble(GenericHID.RumbleType.kBothRumble, 0);
    }
}
