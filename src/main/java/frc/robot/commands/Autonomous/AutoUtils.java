package frc.robot.commands.Autonomous;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Swerve.SwerveDrivetrain;


public class AutoUtils {
    private final static TrajectoryConfig defaultConfig = new TrajectoryConfig(
        AutoConstants.kMaxVelocityMPS, AutoConstants.kMaxAccerlerationMPS)
        .setKinematics(DriveConstants.kDriveKinematics);

    private final static Trajectory defaultTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(), 
        List.of(new Translation2d(20, 0)), 
        new Pose2d(new Translation2d(0, 0), new Rotation2d()), 
        defaultConfig);
    
    public static Command getDefaultTrajectory(SwerveDrivetrain swerve){
        SwerveControllerCommand swerveController = new SwerveControllerCommand(
            defaultTrajectory, 
            swerve::getPose,
            DriveConstants.kDriveKinematics,
            AutoConstants.kXController,
            AutoConstants.kYController,
            AutoConstants.kThetaController,
            swerve::setModuleStates,
            swerve);

            swerve.setPose(defaultTrajectory.getInitialPose());

            return swerveController.andThen(() -> swerve.drive(0, 0, 0));
    }
}
