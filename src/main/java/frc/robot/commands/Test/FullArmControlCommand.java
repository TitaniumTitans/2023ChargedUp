package frc.robot.commands.Test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmSubsystem;


public class FullArmControlCommand extends CommandBase {
    private final ArmSubsystem m_armSubsystem;
    private final double Extension;
    private final double Angle;

    public FullArmControlCommand(ArmSubsystem armSubsystem, double InputExtension, double InputAngle) {
        m_armSubsystem = armSubsystem;
        this.Extension = InputExtension;
        this.Angle = InputAngle;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(m_armSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_armSubsystem.setArmAngle(Angle);
        m_armSubsystem.setArmExtension(Extension);
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
