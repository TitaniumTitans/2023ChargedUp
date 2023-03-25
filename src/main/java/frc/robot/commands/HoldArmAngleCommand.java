package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmAngleSubsystem;


public class HoldArmAngleCommand extends CommandBase {
    private final ArmAngleSubsystem m_angle;
    private double m_staticAngle;


    public HoldArmAngleCommand(ArmAngleSubsystem m_angle) {
        this.m_angle = m_angle;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)

        addRequirements(this.m_angle);
    }

    @Override
    public void initialize() {
        m_staticAngle = m_angle.getArmAngle();
    }
    @Override
    public void execute() {
        m_angle.setArmAngle(m_staticAngle);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }
}
