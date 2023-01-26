package frc.robot.autonomous;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Swerve.SwerveDrivetrain;

public class TrajectoryFactory {
    TrajectoryConfig defaultConfig = 
        new TrajectoryConfig(
            AutoConstants.kMaxVelocityMPS,
            AutoConstants.kMaxAccelerationMPS)
            .setKinematics(DriveConstants.kDriveKinematics);

    Trajectory defaultTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Initial position
            new Pose2d(new Translation2d(0, 0), new Rotation2d(0)),

            List.of(new Translation2d(1, 0), new Translation2d(-2, 0)),

            new Pose2d(new Translation2d(-2, 0), new Rotation2d(0)),
            defaultConfig);

    public Command getDefaultTrajectory(SwerveDrivetrain drive){
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            defaultTrajectory,
            drive::getPose,
            DriveConstants.kDriveKinematics,
            AutoConstants.kControllerX,
            AutoConstants.kControllerY,
            AutoConstants.kThetaController,
            drive::setModuleStates,
            drive);

        drive.resetPose(defaultTrajectory.getInitialPose());

        return swerveControllerCommand.andThen(() -> drive.drive(0, 0, 0));
    }
}
