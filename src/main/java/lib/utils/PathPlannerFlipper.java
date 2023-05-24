package frc.lib.util;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.ArrayList;
import java.util.List;

public class PathPlannerFlipper {
    public static final double FIELD_LENGTH_METERS = Units.inchesToMeters(651.25 - 25); // Area of actual gameplay area, not carpeted area
    public static final double FIELD_WIDTH_METERS = Units.inchesToMeters(315.5);
    private PathPlannerFlipper() {
        new RuntimeException("Utility Class should not be constructed");
    }

    public static PathPlannerTrajectory flipTrajectory(PathPlannerTrajectory trajectory) {
        List<Trajectory.State> origStates = trajectory.getStates();
        List<Trajectory.State> trajStatesNew = new ArrayList<>(List.of());

        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
            for (Trajectory.State state : origStates) {
                trajStatesNew.add(flipState((PathPlannerTrajectory.PathPlannerState) state, DriverStation.getAlliance()));
            }

            return new PathPlannerTrajectory(trajStatesNew,
                    trajectory.getMarkers(),
                    trajectory.getStartStopEvent(),
                    trajectory.getEndStopEvent(),
                    trajectory.fromGUI);
        } else {
            return trajectory;
        }
    }

    public static List<PathPlannerTrajectory> flipTrajectory(List<PathPlannerTrajectory> paths) {
        List<PathPlannerTrajectory> reverse = List.of();

        for (PathPlannerTrajectory p : paths) {
            reverse.add(flipTrajectory(p));
        }

        return reverse;
    }

    public static Trajectory.State flipState(PathPlannerTrajectory.PathPlannerState state, DriverStation.Alliance alliance) {
        if (alliance == DriverStation.Alliance.Red) {
            // Create a new state so that we don't overwrite the original
            PathPlannerTrajectory.PathPlannerState transformedState = new PathPlannerTrajectory.PathPlannerState();

            Translation2d transformedTranslation =
                    new Translation2d(FIELD_LENGTH_METERS - state.poseMeters.getX(), state.poseMeters.getY());

            double headingdif = state.poseMeters.getRotation().getDegrees();
            Rotation2d transformedHeading = Rotation2d.fromDegrees(180 - headingdif);

            double holodif = state.poseMeters.getRotation().getDegrees();
            Rotation2d transformedHolonomicRotation = Rotation2d.fromDegrees(180 - holodif);

            transformedState.timeSeconds = state.timeSeconds;
            transformedState.velocityMetersPerSecond = state.velocityMetersPerSecond;
            transformedState.accelerationMetersPerSecondSq = state.accelerationMetersPerSecondSq;
            transformedState.poseMeters = new Pose2d(transformedTranslation, transformedHeading);
            transformedState.angularVelocityRadPerSec = -state.angularVelocityRadPerSec;
            transformedState.holonomicRotation = transformedHolonomicRotation;
            transformedState.holonomicAngularVelocityRadPerSec = -state.holonomicAngularVelocityRadPerSec;
            transformedState.curvatureRadPerMeter = -state.curvatureRadPerMeter;

            return transformedState;
        } else {
            return state;
        }
    }
}