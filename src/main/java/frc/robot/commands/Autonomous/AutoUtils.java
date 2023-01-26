package frc.robot.commands.Autonomous;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Swerve.SwerveDrivetrain;


public class AutoUtils {
    // Default Constants
    private final static PathConstraints defaultConfig = new PathConstraints(
        AutoConstants.kMaxVelocityMPS, AutoConstants.kMaxAccerlerationMPS);

    private final static PathPlannerTrajectory defaultTrajectory = PathPlanner.generatePath(
        defaultConfig,
        new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(0)), 
        new PathPoint(new Translation2d(Units.feetToMeters(3), Units.feetToMeters(1)), Rotation2d.fromDegrees(0)));
    
    private static final PathPlannerTrajectory defaultAutoGen = PathPlanner.loadPath("DefaultPath", defaultConfig);


    //Default getters
    public static Command getDefaultTrajectory(SwerveDrivetrain swerve){
        return new SequentialCommandGroup(
            new InstantCommand(() -> swerve.setPose(defaultAutoGen.getInitialHolonomicPose())),

            new PPSwerveControllerCommand(defaultAutoGen, 
            swerve::getPose, 
            DriveConstants.kDriveKinematics, 
            AutoConstants.kXController, 
            AutoConstants.kYController, 
            new PIDController(0, 0, 0), 
            swerve::setModuleStates,
            true,
            swerve)
        );
    }

    public static FollowPathWithEvents getPathWithEvents(SwerveDrivetrain swerve){
    HashMap<String, Command> defaultEventMap = new HashMap<>();
    defaultEventMap.put("Event 1", new PrintCommand("Marker 1"));
    defaultEventMap.put("event 2", new PrintCommand("Marker 2"));
    return new FollowPathWithEvents(getDefaultTrajectory(swerve), defaultAutoGen.getMarkers(), defaultEventMap);
    }

    public static Command getAutoRoutine(Path)

    public static Command getAutoEventRoutine(PathPlannerTrajectory traj, HashMap<String, Command> events){

    }
}
