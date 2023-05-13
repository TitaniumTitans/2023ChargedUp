package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.AutoBalanceTransCommand;
import frc.robot.commands.SupersystemToPoseAutoCommand;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.supersystems.ArmPose;
import frc.robot.supersystems.ArmSupersystem;

import java.util.HashMap;
import java.util.List;

public class ScoreTwoBalanceCommandGroup extends SequentialCommandGroup {
    public ScoreTwoBalanceCommandGroup(SwerveDrivetrain m_swerve, ArmSupersystem m_armSupersystem, AutoUtils.ScoringHeights scoreHeight, AutoUtils.StartingZones start) {
        addRequirements(m_swerve);
        m_armSupersystem.addRequirements(this);

        PathConstraints constraints = AutoUtils.getDefaultConstraints();
        DriverStation.Alliance alliance = DriverStation.getAlliance();

        if (scoreHeight == AutoUtils.ScoringHeights.HIGH) {
            addCommands(new SupersystemToPoseAutoCommand(m_armSupersystem, Constants.ArmSetpoints.HIGH_GOAL));
        } else {
            addCommands(new SupersystemToPoseAutoCommand(m_armSupersystem, Constants.ArmSetpoints.MIDDLE_GOAL));
        }

        List<PathPlannerTrajectory> trajectory;

        if((start == AutoUtils.StartingZones.LEFT && alliance == DriverStation.Alliance.Blue) || (start == AutoUtils.StartingZones.RIGHT && alliance == DriverStation.Alliance.Red)) {
            trajectory = PathPlanner.loadPathGroup("PickUp Left Engage", constraints);
        } else if ((start == AutoUtils.StartingZones.RIGHT && alliance == DriverStation.Alliance.Blue) || (start == AutoUtils.StartingZones.LEFT && alliance == DriverStation.Alliance.Red)) {
            trajectory = PathPlanner.loadPathGroup("PickUp Right Engage", constraints);
        } else {
            trajectory = PathPlanner.loadPathGroup("PickUp Middle Engage", constraints);
        }

        ArmPose armScoringPose;

        // We don't score low (for now at least)
        if (scoreHeight == AutoUtils.ScoringHeights.MIDDLE) {
            armScoringPose = Constants.ArmSetpoints.MIDDLE_GOAL;
        } else {
            armScoringPose = Constants.ArmSetpoints.HIGH_GOAL;
        }

        HashMap<String, Command> autoEvents = new HashMap<>();
        autoEvents.put("LowerIntake", (new SupersystemToPoseAutoCommand(m_armSupersystem, Constants.ArmSetpoints.INTAKE_BATTERY))
                .andThen(m_armSupersystem.runIntakeForTime(0.75, 1.0))
                .andThen(new SupersystemToPoseAutoCommand(m_armSupersystem, Constants.ArmSetpoints.STOW_POSITION)));
        autoEvents.put("ClearGround", (new SupersystemToPoseAutoCommand(m_armSupersystem, Constants.ArmSetpoints.STOW_POSITION)));
        autoEvents.put("Score", 
                (new SupersystemToPoseAutoCommand(m_armSupersystem, armScoringPose))
                .andThen(m_armSupersystem.runIntakeForTime(0.09, -0.6)));
        autoEvents.put("RaiseArm",
            new SupersystemToPoseAutoCommand(m_armSupersystem, armScoringPose));

        addCommands(new SupersystemToPoseAutoCommand(m_armSupersystem, armScoringPose));
        addCommands(m_armSupersystem.runIntakeForTime(0.15, -0.6));
        addCommands(m_swerve.getAutoBuilder(autoEvents).fullAuto(trajectory).andThen(new AutoBalanceTransCommand(m_swerve)));
    }



}

