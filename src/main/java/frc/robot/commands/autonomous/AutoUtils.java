package frc.robot.commands.autonomous;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrain;


public class AutoUtils {

    private AutoUtils() {
        throw new IllegalStateException("Utility Class");
    }

    // Default Constants
    private static final PathConstraints m_defaultConfig = new PathConstraints(
        AutoConstants.MAX_VELOCITY_PERCENT_OUTPUT, AutoConstants.MAX_ACCELERATION_PERCENT_OUTPUT);
        

    private static final PathPlannerTrajectory m_defaultAutoGen = PathPlanner.loadPath("DefaultPath", m_defaultConfig);

    public static PathConstraints getDefaultConstraints() {
        return m_defaultConfig;
    }
    //Default getters
    public static Command getDefaultTrajectory(SwerveDrivetrain swerve) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> swerve.resetPose(m_defaultAutoGen.getInitialHolonomicPose())),

            new PPSwerveControllerCommand(m_defaultAutoGen, 
            swerve::getPose, 
            DriveConstants.DRIVE_KINEMATICS, 
            AutoConstants.CONTROLLER_X, 
            AutoConstants.CONTROLLER_Y, 
            new PIDController(0, 0, 0), 
            swerve::setModuleStates,
            true,
            swerve)
        );
    }

    public static FollowPathWithEvents getPathWithEvents(SwerveDrivetrain swerve) {
    HashMap<String, Command> defaultEventMap = new HashMap<>();
    defaultEventMap.put("Event 1", new PrintCommand("Marker 1"));
    defaultEventMap.put("event 2", new PrintCommand("Marker 2"));
    return new FollowPathWithEvents(getDefaultTrajectory(swerve), m_defaultAutoGen.getMarkers(), defaultEventMap);
    }

    public static Command getAutoRoutine(PathPlannerTrajectory traj, SwerveDrivetrain swerve){
        return new SequentialCommandGroup(
            new InstantCommand(() -> swerve.resetPose(traj.getInitialHolonomicPose())),

            new PPSwerveControllerCommand(traj,
            swerve::getPose, 
            DriveConstants.DRIVE_KINEMATICS, 
            AutoConstants.CONTROLLER_X, 
            AutoConstants.CONTROLLER_Y, 
            new PIDController(0, 0, 0), 
            swerve::setModuleStates,
            true,
            swerve)
        );
    }
    
    public static Command getAutoEventRoutine(PathPlannerTrajectory traj, Map<String, Command> events, SwerveDrivetrain swerve) {
        return new FollowPathWithEvents(getAutoRoutine(traj, swerve), traj.getMarkers(), events);
    }
}
