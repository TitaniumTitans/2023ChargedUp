package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.supersystems.ArmSupersystem;


public class ArmToScoreCommand extends CommandBase {

    private final ArmSupersystem m_super;

    private final double m_heading;

    public ArmToScoreCommand(ArmSupersystem supersystem, SwerveDrivetrain drive) {
        m_super = supersystem;
        m_heading = drive.getGyroYaw().getDegrees();
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        m_super.addRequirements(this);
    }

    @Override
    public void execute() {
        if (m_heading > 270 && m_heading < 90) {
            // If the front of the robot is pointing toward the scoring zone
            m_super.setToPose(Constants.ArmSetpoints.HIGH_GOAL);
        } else if (m_heading <= 270 && m_heading >= 90) {
            // If the stow side of the robot is pointing toward the scoring zone
            m_super.setToPose(Constants.ArmSetpoints.MIDDLE_GOAL_STOW);
        } else {
            m_super.setToPose(Constants.ArmSetpoints.STOW_POSITION);
        }
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

}
