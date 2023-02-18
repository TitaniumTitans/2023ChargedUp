package frc.robot.commands.nosupersystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmAngleSubsystem;
import frc.robot.subsystems.arm.ArmExtSubsystem;


public class FullArmControlCommand extends CommandBase {
    private final ArmAngleSubsystem m_armAngleSubsystem;
    private final ArmExtSubsystem m_armExtSubsystem;
    private final double extension;
    private final double angle;

    public FullArmControlCommand(ArmAngleSubsystem armAngleSubsystem, ArmExtSubsystem armExt, double inputExtension, double inputAngle) {
        m_armAngleSubsystem = armAngleSubsystem;
        m_armExtSubsystem = armExt;
        this.extension = inputExtension;
        this.angle = inputAngle;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(m_armAngleSubsystem);
    }

    @Override
    public void execute() {
        m_armAngleSubsystem.setArmAngle(angle);
        m_armExtSubsystem.setArmExtension(extension);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_armExtSubsystem.setArmSpeed(0.0);
        m_armAngleSubsystem.setAngleSpeed(0.0);
    }
}
