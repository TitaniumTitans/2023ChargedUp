package lib.utils.zone

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Transform2d
import lib.utils.piecewise.Range

class Zone(topleft: Transform2d, width: Double, height: Double) {

    val leftRange: Range
    val topRange: Range
    init {
        //Construct a "rectangle" for zone based off the uppermost left point
        //Make sure all units are in meters
        leftRange = Range(topleft.x, true, topleft.y - height, true)
        topRange = Range(topleft.y, true, topleft.x - width, true)
    }

    fun inZone(pose: Pose2d): Boolean {
        var inLeft = leftRange.withinRange(pose.y)
        var inTop = topRange.withinRange(pose.x)

        return inLeft && inTop
    }
}