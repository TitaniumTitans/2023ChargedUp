package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.commands.autonomous.AutoUtils;
import frc.robot.subsystems.swerve.SwerveDrivetrain;


public class AutoAlignCommand extends CommandBase {

    public static enum AlignmentOptions {
        LEFT_ALIGN,
        CENTER_ALIGN,
        RIGHT_ALIGN,
        HUMAN_PLAYER_ALIGN
    }

    private final SwerveDrivetrain m_drive;
    private PathPlannerTrajectory m_traj;
    private Command path;

    private final Transform2d centerTranslation = new Transform2d(
            new Translation2d(0.6, 0.0),
            new Rotation2d(0.0)
    );

    private  final Transform2d leftTranslation = new Transform2d(
            new Translation2d(0.6, 0.6),
            new Rotation2d()
    );

    private final Transform2d rightTranslation = new Transform2d(
            new Translation2d(0.6, -0.6),
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

        Translation2d translatedEnd;
        Translation2d translatedMiddle;

        switch(m_align){
            case HUMAN_PLAYER_ALIGN:
            case LEFT_ALIGN:
                translatedEnd = tagPose.transformBy(leftTranslation).getTranslation();
                break;
            case RIGHT_ALIGN:
                translatedEnd = tagPose.transformBy(rightTranslation).getTranslation();
                break;
            default:
                translatedEnd = tagPose.transformBy(centerTranslation).getTranslation();
        }

        translatedMiddle = tagPose.getRotation().getDegrees() == 180 ?
                translatedEnd.minus(new Translation2d(0.25, 0)) :
                translatedEnd.plus(new Translation2d(0.25, 0));


        //generate a path based on the tag you see, flipped 180 from tag pose
        m_traj = PathPlanner.generatePath(
                AutoUtils.getDefaultConstraints(),
                new PathPoint(m_drive.getPose().getTranslation(), new Rotation2d(), m_drive.getPose().getRotation()),
                new PathPoint(translatedMiddle, new Rotation2d(), tagPose.getRotation().rotateBy(Rotation2d.fromDegrees(180))),
                new PathPoint(translatedEnd, new Rotation2d(), tagPose.getRotation().rotateBy(Rotation2d.fromDegrees(180)))
        );

//        SmartDashboard.putNumber("Tag Rotation", tagPose.getRotation().getDegrees());

        m_drive.createPPSwerveController(m_traj);
//        path = AutoUtils.getAutoRoutine(m_traj, m_drive, false);
//        path.schedule();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
//        path.cancel();
    }
}
