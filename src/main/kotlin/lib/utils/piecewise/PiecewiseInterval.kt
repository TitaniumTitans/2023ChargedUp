package lib.utils.piecewise

import java.lang.RuntimeException
import java.util.function.DoubleFunction

/**
 *
 * @param range The range from which this interval will produce an output for
 * @param function Must be valid from `range.left` to `range.right` inclusive
 *
 */
class PiecewiseInterval<T>(val range: Range, private val function: DoubleFunction<T>) {

    init {
        // 1. Check to make sure left is smaller than right.
        // 2. In the case where left and right are equal, make sure left and right are inclusive
        //    (This is an interval of a single point)
        val left = range.left
        val right = range.right
        val leftInclusive = range.leftInclusive
        val rightInclusive = range.rightInclusive
        if (/* 1 */(left > right) || /* 2 */((left == right) && !(leftInclusive && rightInclusive))) {
            throw RuntimeException("Invalid Interval")
        }
    }

    /**
     * Check if a value is within the range of the piecewise
     */
    fun isInRange(input: Double): Boolean {
        return isInLeftRange(input) && isInRightRange(input)
    }

    fun isInLeftRange(input: Double): Boolean {
        return range.withinLeftRange(input)
    }

    fun isInRightRange(input: Double): Boolean {
        return range.withinRightRange(input)
    }

    fun calculate(input: Double): T {
        if (isInRange(input)) {
            return function.apply(input)
        } else {
            /*  Example string: "Input 10 is not in range (1, 5)"
                Example string: "Input 10 is not in range [1, 5)"
                Example string: "Input 10 is not in range (1, 5]"
                Example string: "Input 10 is not in range [1, 5]"
             */
            // TODO: Decide what to do here.
            throw RuntimeException("Input $input is not in range ${if (range.leftInclusive) '[' else '('}$range.left, $range.right${if (range.rightInclusive) ']' else ')'}")
        }
    }
}
