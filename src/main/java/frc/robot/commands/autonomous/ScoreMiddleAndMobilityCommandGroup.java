package frc.robot.commands.autonomous;


import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.supersystems.ArmSupersystem;

import java.util.HashMap;

public class ScoreMiddleAndMobilityCommandGroup extends SequentialCommandGroup {
    public ScoreMiddleAndMobilityCommandGroup(SwerveDrivetrain drive, ArmSupersystem arm, WristSubsystem wrist) {
        PathPlannerTrajectory scoreTraj = PathPlanner.loadPath("Score Cone Mobility", AutoUtils.getDefaultConstraints());
        addCommands(new ScoreMiddleCommandGroup(arm, wrist)); // Score first piece

        addCommands(drive.getAutoBuilder(new HashMap<>()).fullAuto(scoreTraj)); // Move out of starting zone
    }
}