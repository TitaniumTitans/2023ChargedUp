package lib.utils.piecewise

import java.lang.RuntimeException

class RangedPiecewise<T>(private val range: Range, private var intervalList: List<PiecewiseInterval<T>> = ArrayList()) {

    private var hasBeenVerified = false

    init {
        intervalList = intervalList.sortedBy {it.range.left}
    }
    fun verifyRange() : Boolean {
        var currentPosition = range.left
        var currentPositionInclusive = range.leftInclusive
        for (interval in intervalList) {
            if(currentPositionInclusive == interval.range.leftInclusive && currentPosition == interval.range.left) {
                currentPosition = interval.range.right
                currentPositionInclusive = !interval.range.rightInclusive
            } else {
                hasBeenVerified = false
                return false
            }
        }
        hasBeenVerified = true
        return true
    }

    fun calculate(input: Double): T {
        // Check the cached value first, then verify if needed
        if(!hasBeenVerified && !verifyRange()) {
                throw RuntimeException("Range is not valid")
        }
        return intervalList.find { interval -> interval.isInRange(input) }?.calculate(input) ?: throw RuntimeException("An interval should have been found. Please report this issue.")
    }

}