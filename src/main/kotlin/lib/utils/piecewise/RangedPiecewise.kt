package lib.utils.piecewise

import java.lang.RuntimeException
import java.util.*

class RangedPiecewise<T>(
    private val range: Range,
    private var intervalList: List<PiecewiseInterval<T>> = Collections.emptyList()
) {

    init {
        intervalList = intervalList.sortedBy { it.range.left }
        check(verifyRange()) { "Provided list is not valid" }
    }

    private fun verifyRange(): Boolean {
        var currentPosition = range.left
        var currentPositionInclusive = range.leftInclusive
        for (interval in intervalList) {
            if (currentPositionInclusive == interval.range.leftInclusive && currentPosition == interval.range.left) {
                currentPosition = interval.range.right
                currentPositionInclusive = !interval.range.rightInclusive
            } else {
                return false
            }
        }
        return true
    }

    fun calculate(input: Double): T {
        // Check the cached value first, then verify if needed
        return intervalList.find { interval -> interval.isInRange(input) }?.calculate(input)
            ?: throw RuntimeException("An interval should have been found. Please report this issue.")
    }
}
