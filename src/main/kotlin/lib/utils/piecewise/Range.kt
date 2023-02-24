package lib.utils.piecewise

import edu.wpi.first.math.MathUtil
import kotlin.math.nextDown
import kotlin.math.nextUp

/**
 * @param left The lesser value
 * @param leftInclusive Whether the lesser extreme is included in the range
 * @param right The greater value
 * @param rightInclusive Whether the greater extreme is included in the range
 *
 */

data class Range(var left: Double, var leftInclusive: Boolean, var right: Double, var rightInclusive: Boolean) {
    fun withinRange(isIn: Double): Boolean {
        return ((left < isIn) || (leftInclusive && isIn == left)) || ((right > isIn) || (rightInclusive && isIn == right))
    }

    fun withinLeftRange(input: Double): Boolean {
        return (left < input) || (leftInclusive && left == input)
    }

    fun withinRightRange(input: Double): Boolean {
        return (right > input) || (rightInclusive && right == input)
    }

    fun clamp(input: Double): Double {
        return MathUtil.clamp(input,
            // If left is not inclusive, bring it slightly
            if(leftInclusive) left else left.nextUp(),
            // If right is not inclusive, bit it down slightly
            if(rightInclusive) right else right.nextDown()
        )
    }

    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false

        other as Range

        if (left != other.left) return false
        if (leftInclusive != other.leftInclusive) return false
        if (right != other.right) return false
        if (rightInclusive != other.rightInclusive) return false

        return true
    }

    override fun hashCode(): Int {
        var result = left.hashCode()
        result = 31 * result + leftInclusive.hashCode()
        result = 31 * result + right.hashCode()
        result = 31 * result + rightInclusive.hashCode()
        return result
    }

}
