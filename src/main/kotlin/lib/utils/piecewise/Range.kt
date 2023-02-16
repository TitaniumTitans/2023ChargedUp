package lib.utils.piecewise

/**
 * @param left The lesser value
 * @param leftInclusive Whether the lesser extreme is included in the range
 * @param right The greater value
 * @param rightInclusive Whether the greater extreme is included in the range
 *
 */
data class Range(val left: Double, val leftInclusive: Boolean, val right: Double, val rightInclusive: Boolean)
