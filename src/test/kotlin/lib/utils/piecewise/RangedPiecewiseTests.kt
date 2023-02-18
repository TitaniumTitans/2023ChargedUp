package lib.utils.piecewise

import org.junit.jupiter.api.Test
import org.junit.jupiter.api.assertThrows
import java.lang.IllegalStateException
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
}