package lib.utils.zone

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation

class FieldZones {
    enum class ZoneLocations {
        SCORE_LEFT,
        SCORE_RIGHT,
        SCORE_MIDDLE,
        PLAYER_STATION,
        OPPOSING_SCORING,
        FIELD
    }

    companion object {
        // Zones defining the field
        val m_blueScoreLeft = Zone(
            Translation2d(0.0, 0.0),
            Translation2d(1.0, 1.0)
        )

        val m_blueScoreMiddle = Zone(
            Translation2d(1.0, 1.0),
            Translation2d(2.0, 2.0)
        )

        fun getZone(robotPose: Pose2d): ZoneLocations {
            if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
                // Check for zone positions while on blue alliance
                if (m_blueScoreLeft.inZone(robotPose)) {
                    return ZoneLocations.SCORE_LEFT
                } else if (m_blueScoreMiddle.inZone(robotPose)) {
                    return ZoneLocations.SCORE_MIDDLE
                }
            } else if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {}
            // If not inside a zone return FIELD location
            return ZoneLocations.FIELD
        }
    }
}
