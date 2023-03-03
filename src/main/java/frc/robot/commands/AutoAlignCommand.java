package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.autonomous.AutoUtils;
import frc.robot.subsystems.swerve.SwerveDrivetrain;


public class AutoAlignCommand extends CommandBase {
    private final SwerveDrivetrain m_drive;

    private final Pose2d m_pose;
    private PathPlannerTrajectory m_traj;
    private Command path;
    public AutoAlignCommand(SwerveDrivetrain swerveDrivetrain, Pose2d pose) {
        this.m_drive = swerveDrivetrain;
        m_pose = pose;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.m_drive);
    }

    @Override
    public void initialize() {
        m_traj = PathPlanner.generatePath(
                AutoUtils.getDefaultConstraints(),
                new PathPoint(m_drive.getPose().getTranslation(), m_drive.getPose().getRotation()),
                new PathPoint(m_pose.getTranslation(), m_pose.getRotation())
        );

        path = AutoUtils.getAutoRoutine(m_traj, m_drive);
        CommandScheduler.getInstance().schedule(path);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        CommandScheduler.getInstance().cancel(path);
    }
}
