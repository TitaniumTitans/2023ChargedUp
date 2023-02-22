package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.wrist.WristSubsystem;


public class IntakeControlCommand extends CommandBase {

    WristSubsystem m_wrist;

    double m_speed;

    public IntakeControlCommand(WristSubsystem wrist, double speed) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        m_wrist = wrist;
        m_speed = speed;
        addRequirements();
    }

    @Override
    public void execute() {
        m_wrist.setIntakeSpeed(m_speed);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_wrist.setIntakeSpeed(0.0);
    }
}
