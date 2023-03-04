package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.autonomous.AutoUtils;
import frc.robot.subsystems.swerve.SwerveDrivetrain;


public class AutoAlignCommand extends CommandBase {

    public static enum AlignmentOptions {
        LEFT_ALIGN,
        CENTER_ALIGN,
        RIGHT_ALIGN
    }

    private final SwerveDrivetrain m_drive;
    private PathPlannerTrajectory m_traj;
    private Command path;

    private final Transform2d centerTranslation = new Transform2d(
            new Translation2d(0.55, 0.0),
            new Rotation2d(0.0)
    );

    private  final Transform2d leftTranslation = new Transform2d(
            new Translation2d(0.55, 0.5),
            new Rotation2d()
    );

    private final Transform2d rightTranslation = new Transform2d(
            new Translation2d(0.55, -0.5),
            new Rotation2d()
    );

    private AlignmentOptions m_align = AlignmentOptions.CENTER_ALIGN;

    public AutoAlignCommand(SwerveDrivetrain swerveDrivetrain, AlignmentOptions aligment) {
        this.m_drive = swerveDrivetrain;
        m_align = aligment;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.m_drive);
    }

    @Override
    public void initialize() {

        // Figure out what pose the robot should be
        Pose2d tagPose = m_drive.getFrontCamTagPose();

        Translation2d translatedTranslation;

        switch(m_align){
            case LEFT_ALIGN:
                translatedTranslation = tagPose.transformBy(leftTranslation).getTranslation();
                break;
            case RIGHT_ALIGN:
                translatedTranslation = tagPose.transformBy(rightTranslation).getTranslation();
                break;
            default:
                translatedTranslation = tagPose.transformBy(centerTranslation).getTranslation();
        }


        //generate a path based on the tag you see, flipped 180 from tag pose
        m_traj = PathPlanner.generatePath(
                AutoUtils.getDefaultConstraints(),
                new PathPoint(m_drive.getPose().getTranslation(), m_drive.getPose().getRotation()),
                new PathPoint(translatedTranslation.minus(new Translation2d(0.25, 0)), tagPose.getRotation()),
                new PathPoint(translatedTranslation, tagPose.getRotation())
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
