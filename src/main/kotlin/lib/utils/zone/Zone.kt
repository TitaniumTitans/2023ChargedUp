package lib.utils.zone

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import lib.utils.piecewise.Range

class Zone(topleft: Translation2d, bottomRight: Translation2d) {

    val leftRange: Range
    val topRange: Range
    init {
        // Construct a "rectangle" for zone based off the uppermost left point
        // Make sure all units are in meters
        leftRange = Range(topleft.y, true, bottomRight.y, true)
        topRange = Range(topleft.x, true, bottomRight.x, true)
    }

    fun inZone(pose: Pose2d): Boolean {
        var inLeft = leftRange.withinRange(pose.y)
        var inTop = topRange.withinRange(pose.x)

        return inLeft && inTop
    }
}
