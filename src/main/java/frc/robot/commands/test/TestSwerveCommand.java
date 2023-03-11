package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDrivetrain;


public class TestSwerveCommand extends CommandBase {

    private double m_angle;
    private SwerveDrivetrain m_drive;

    public TestSwerveCommand(SwerveDrivetrain drive, double angle) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        m_angle = angle;
        m_drive = drive;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (0 <= m_angle && m_angle <= 89){
            m_drive.drive(0.5, 0.0, 0.0);
        } else if ((90 <= m_angle) && m_angle <= 179) {
            m_drive.drive(0.0, 0.5, 0.0);
        } else if ((180 <= m_angle) && (m_angle <= 269)) {
            m_drive.drive(-0.5, 0.0, 0.0);
        } else if (270 <= m_angle && m_angle <= 360) {
            m_drive.drive(0.0, -0.5, 0.0);
        }
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
