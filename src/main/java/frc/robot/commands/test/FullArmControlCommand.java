package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmSubsystem;


public class FullArmControlCommand extends CommandBase {
    private final ArmSubsystem m_armSubsystem;
    private final double extension;
    private final double angle;

    public FullArmControlCommand(ArmSubsystem armSubsystem, double inputExtension, double inputAngle) {
        m_armSubsystem = armSubsystem;
        this.extension = inputExtension;
        this.angle = inputAngle;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(m_armSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_armSubsystem.setArmAngle(angle);
        m_armSubsystem.setArmExtension(extension);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_armSubsystem.setArmSpeed(0.0);
        m_armSubsystem.setAngleSpeed(0.0);
    }
}
