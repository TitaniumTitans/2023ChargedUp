package frc.robot.commands.autonomous;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrain;


public class AutoUtils {

    public enum ScoringHeights {
        HIGH,
        MIDDLE,
        LOW
    }

    public enum StartingZones {
        LEFT,
        MIDDLE,
        RIGHT
    }

    private AutoUtils() {
        throw new IllegalStateException("Utility Class");
    }

    // Default Constants
    private static final PathConstraints m_defaultConfig = new PathConstraints(
        AutoConstants.MAX_VELOCITY_MPS_AUTO, AutoConstants.MAX_ACCELERATION_MPS_AUTO);
        

//    private static final PathPlannerTrajectory m_defaultAutoGen = PathPlanner.loadPath("DefaultPath", m_defaultConfig);

//    private static final SwerveAutoBuilder defaultAutoFactory = new SwerveAutoBuilder(
//
//    )

    public static PathConstraints getDefaultConstraints() {
        return m_defaultConfig;
    }

    public static Command getAutoRoutine(PathPlannerTrajectory traj, SwerveDrivetrain swerve, boolean firstTrajectory){
        return new SequentialCommandGroup(
            new ConditionalCommand(new InstantCommand(() ->
                    swerve.resetPose(
                    PathPlannerTrajectory.transformTrajectoryForAlliance(traj, DriverStation.getAlliance()).getInitialHolonomicPose())),
                    new InstantCommand(),
                    () -> firstTrajectory
            ),


            new PPSwerveControllerCommand(traj,
            swerve::getPose, 
            DriveConstants.DRIVE_KINEMATICS, 
            AutoConstants.CONTROLLER_X, 
            AutoConstants.CONTROLLER_Y, 
            AutoConstants.THETA_CONTROLLER,
            swerve::setModuleStates,
            true,
            swerve)
        );
    }
    
    public static Command getAutoEventRoutine(PathPlannerTrajectory traj, Map<String, Command> events, SwerveDrivetrain swerve) {
        return new FollowPathWithEvents(getAutoRoutine(traj, swerve, true), traj.getMarkers(), events);
    }
}
