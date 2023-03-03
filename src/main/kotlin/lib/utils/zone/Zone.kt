package lib.utils.zone

import edu.wpi.first.math.geometry.Transform2d
import lib.utils.piecewise.Range

class Zone(topleft: Transform2d, topright: Transform2d, bottomleft: Transform2d, bottomright: Transform2d) {

    init {
        // create a "square" on the field to check if
        val topRange = Range(topleft.x, true, topright.x, true)
        val bottomRange = Range(bottomleft.x, true, bottomright.x, true)
        val leftRange = Range(topleft.y, true, bottomleft.y, true)
        val rightRange = Range(topright.y, true, bottomright.y, true)
    }
}