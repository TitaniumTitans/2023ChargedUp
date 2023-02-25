package lib.utils.piecewise

import frc.robot.Constants.LimitConstants
import frc.robot.supersystems.ArmLimits
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.assertThrows
import kotlin.test.junit5.JUnit5Asserter.assertEquals

class RangedPiecewiseTests {
    private val testingRange = Range(0.0, true, 15.0, false)
    private val goodRange =
        listOf(
            PiecewiseInterval(Range(0.0, true, 5.0, false)) { input -> input * 2 },
            PiecewiseInterval(Range(5.0, true, 10.0, false)) { _ -> 42.0 },
            PiecewiseInterval(Range(10.0, true, 15.0, false)) { input -> input + 5 }
        )

    private val goodRangeWithSinglePoint =
        listOf(
            PiecewiseInterval(Range(0.0, true, 5.0, false)) { input -> input * 2 },
            PiecewiseInterval(Range(5.0, true, 5.0, true)) { _ -> 21.0 },
            PiecewiseInterval(Range(5.0, false, 10.0, false)) { _ -> 42.0 },
            PiecewiseInterval(Range(10.0, true, 15.0, false)) { input -> input + 5 }
        )

    @Test
    fun testGoodRange() {
        val goodPiecewise = RangedPiecewise(testingRange, goodRange)
        assertEquals("Check for inclusive extreme", goodPiecewise.calculate(0.0), 0.0)
        assertEquals("Check for inclusive and exclusive overlap", goodPiecewise.calculate(5.0), 42.0)
        assertEquals("Check for middle of interval", goodPiecewise.calculate(12.0), 12.0 + 5)
    }

    @Test
    fun testRangeWithSinglePoint() {
        val goodPiecewiseWithSinglePoint = RangedPiecewise(testingRange, goodRangeWithSinglePoint)
        assertEquals("Check the single point", goodPiecewiseWithSinglePoint.calculate(5.0), 21.0)
        assertEquals("Check in an interval", goodPiecewiseWithSinglePoint.calculate(5.1), 42.0)
    }

    private val badRangeExcludingMiddlePoint =
        listOf(
            PiecewiseInterval(Range(0.0, true, 5.0, false)) { input -> input * 2 },
            PiecewiseInterval(Range(5.0, false, 10.0, false)) { _ -> 0.0 },
            PiecewiseInterval(Range(10.0, true, 15.0, false)) { input -> input + 5 }
        )

    @Test
    fun testBadRange() {
        assertThrows<IllegalStateException> { RangedPiecewise(testingRange, badRangeExcludingMiddlePoint) }
    }

    @Test
    fun testRobotCode() {
        val fullArmRange = Range(LimitConstants.STOW_ZONE.value, true, LimitConstants.MAX_MOVEMENT.value, false)
        val stowRange = Range(LimitConstants.STOW_ZONE.value, true, LimitConstants.SCORE_ZONE.value, false)
        val fullRangeZone = Range(LimitConstants.SCORE_ZONE.value, true, LimitConstants.GROUND_ZONE.value, false)
        val floorCollisionZone = Range(LimitConstants.GROUND_ZONE.value, true, LimitConstants.MAX_MOVEMENT.value, false)

        val stowArmLimits = ArmLimits(
                LimitConstants.WRIST_STOW.value, LimitConstants.WRIST_STOW.value,
                LimitConstants.ARM_EXT_STOW.value, LimitConstants.ARM_EXT_STOW.value,
                LimitConstants.ARM_ANGLE_LOWER.value, LimitConstants.ARM_ANGLE_UPPER.value
            )
        val fullArmLimits = ArmLimits(
            LimitConstants.WRIST_SCORE_LOWER.value, LimitConstants.WRIST_SCORE_UPPER.value,
            LimitConstants.ARM_EXT_SCORE_LOWER.value, LimitConstants.ARM_EXT_SCORE_UPPER.value,
            LimitConstants.ARM_ANGLE_LOWER.value, LimitConstants.ARM_ANGLE_UPPER.value
        )
        val floorCollisionLimits = ArmLimits(
            LimitConstants.WRIST_STOW.value, LimitConstants.WRIST_STOW.value,
            LimitConstants.ARM_EXT_STOW.value, LimitConstants.ARM_EXT_STOW.value,
            LimitConstants.ARM_ANGLE_LOWER.value, LimitConstants.ARM_ANGLE_UPPER.value
        )

        val armRegion = listOf(
            PiecewiseInterval( stowRange ) { _ -> stowArmLimits },
            PiecewiseInterval( fullRangeZone ) { _ -> fullArmLimits },
            PiecewiseInterval( floorCollisionZone ) { _ -> floorCollisionLimits}
        )
        val limitPiecewise = RangedPiecewise(fullArmRange, armRegion)

        assertEquals("Left extreme is clamped", limitPiecewise.calculate(fullArmRange.left - 1), limitPiecewise.calculate(fullArmRange.left))
        assertEquals("Right extreme is clamped", limitPiecewise.calculate(fullArmRange.right + 1), limitPiecewise.calculate(fullArmRange.right))
    }

}
