package lib.utils.piecewise

import java.lang.RuntimeException
import java.util.*

/**
 * @param piecewiseRange The entire range of the piecewise
 * @param intervalList An unsorted list of PiecewiseIntervals which will cover the entire range `piecewiseRange`
 */
class RangedPiecewise<T>(
    private val piecewiseRange: Range,
    private var intervalList: List<PiecewiseInterval<T>> = Collections.emptyList()
) {

    init {
        intervalList = intervalList.sortedBy { it.range.left }
        check(verifyRange()) { "Provided list is not valid" }
    }

    private fun verifyRange(): Boolean {
        var currentPosition = piecewiseRange.left
        var currentPositionInclusive = piecewiseRange.leftInclusive
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
        val clampedInput = piecewiseRange.clamp(input)
        // Check the cached value first, then verify if needed
        return intervalList.find { interval -> interval.isInRange(clampedInput) }?.calculate(clampedInput)
            ?: throw RuntimeException("An interval should have been found. Please report this issue.")
    }

    fun clamp(input: Double): Double {
        return piecewiseRange.clamp(input)
    }

}
