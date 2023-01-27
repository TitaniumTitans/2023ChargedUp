package frc.robot.commands.Autonomous;

import java.util.HashMap;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

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
import frc.robot.subsystems.Swerve.SwerveDrivetrain;


public class AutoFactory {
    // Default Constants
    private final static PathConstraints defaultConfig = new PathConstraints(
        AutoConstants.kMaxVelocityMPS, AutoConstants.kMaxAccelerationMPS);
    private static final PathPlannerTrajectory defaultAutoGen = PathPlanner.loadPath("DefaultPath", defaultConfig);
    private static final HashMap<String, Command> defaultEventMap = new HashMap<>();

    private final SwerveDrivetrain m_swerveInstance;

    //Sendable chooser
    private LoggedDashboardChooser<Command> m_chooser;

    public AutoFactory(SwerveDrivetrain swerve){
        m_swerveInstance = swerve;

        defaultEventMap.put("Event 1", new PrintCommand("Marker 1"));
        defaultEventMap.put("event 2", new PrintCommand("Marker 2"));

        m_chooser = new LoggedDashboardChooser<>("Auto Routines");

        configSendableChooser();
    }

    private void configSendableChooser(){
        //Default options, maybe remove before comps
        m_chooser.addDefaultOption("Default Option", getDefaultTrajectory(m_swerveInstance));
        m_chooser.addOption("Default Events", getDefaultPathWithEvents(m_swerveInstance));
    }

    public Command getAuto(){
        return m_chooser.get();
    }

    //Default getters
    public static Command getDefaultTrajectory(SwerveDrivetrain swerve){
        return getAutoRoutine(defaultAutoGen, swerve);
    }

    public static FollowPathWithEvents getDefaultPathWithEvents(SwerveDrivetrain swerve){
    HashMap<String, Command> defaultEventMap = new HashMap<>();
    defaultEventMap.put("Event 1", new PrintCommand("Marker 1"));
    defaultEventMap.put("event 2", new PrintCommand("Marker 2"));
    return new FollowPathWithEvents(getDefaultTrajectory(swerve), defaultAutoGen.getMarkers(), defaultEventMap);
    }

    public static Command getAutoRoutine(PathPlannerTrajectory traj, SwerveDrivetrain swerve){
        return new SequentialCommandGroup(
            new InstantCommand(() -> swerve.resetPose(defaultAutoGen.getInitialHolonomicPose())),

            new PPSwerveControllerCommand(defaultAutoGen, 
            swerve::getPose, 
            DriveConstants.kDriveKinematics, 
            AutoConstants.kControllerX, 
            AutoConstants.kControllerY, 
            new PIDController(0, 0, 0), 
            swerve::setModuleStates,
            true,
            swerve)
        );
    }
    
    public static Command getAutoEventRoutine(PathPlannerTrajectory traj, HashMap<String, Command> events, SwerveDrivetrain swerve){
        return new FollowPathWithEvents(getAutoRoutine(traj, swerve), traj.getMarkers(), events);
    }
}
