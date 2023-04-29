package frc.robot.commands.autonomous;


import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.AutoBalanceTransCommand;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

import java.util.HashMap;

public class BalanceCommandGroup extends SequentialCommandGroup {
    public BalanceCommandGroup(SwerveDrivetrain m_swerve, AutoUtils.StartingZones m_start) {
        DriverStation.Alliance alliance = DriverStation.getAlliance();
        PathPlannerTrajectory trajectory;
        PathConstraints constraints = new PathConstraints(Units.feetToMeters(14), Units.feetToMeters(14) / 3);

        switch(m_start) {
            case LEFT:
                if (alliance == DriverStation.Alliance.Blue) {
                    trajectory = PathPlanner.loadPath("Balance Left", constraints);
                } else {
                    trajectory = PathPlanner.loadPath("Balance Right", constraints);
                }
                break;
            case RIGHT:
                if (alliance == DriverStation.Alliance.Blue) {
                    trajectory = PathPlanner.loadPath("Balance Right", constraints);
                } else {
                    trajectory = PathPlanner.loadPath("Balance Left", constraints);
                }
                break;

            case MIDDLE:
                trajectory = PathPlanner.loadPath("Balance Middle", constraints);
                break;
            default:
                trajectory = new PathPlannerTrajectory();
        }

        addCommands(m_swerve.getAutoBuilder(new HashMap<>()).fullAuto(trajectory));
        addCommands(new AutoBalanceTransCommand(m_swerve));
    }
}