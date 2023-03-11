package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.supersystems.ArmPose;
import frc.robot.supersystems.ArmSupersystem;


public class SupersystemToPoseAutoCommand extends CommandBase {

    ArmSupersystem m_supersystem;
    ArmPose m_pose;

    public SupersystemToPoseAutoCommand(ArmSupersystem supersystem, ArmPose pose) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)

        m_supersystem = supersystem;
        m_pose = pose;

        m_supersystem.addRequirements(this);
    }

    @Override
    public void initialize() {
        m_supersystem.stopSpeed();
    }

    @Override
    public void execute() {
        m_supersystem.setToPose(m_pose);
        SmartDashboard.putBoolean("Auto At Setpoint", m_supersystem.atSetpoint());
    }

    @Override
    public boolean isFinished() {
        return m_supersystem.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        m_supersystem.stopSpeed();
    }
}
