package lib.utils.piecewise

import kotlin.test.*
import kotlin.test.junit5.JUnit5Asserter.assertEquals

class RangedPiecewiseTest {
    private val testingRange = Range(0.0, true, 15.0, false)
    private val goodRange =
                listOf(
                    PiecewiseInterval(Range(0.0, true, 5.0, false)) { input -> input * 2 },
                    PiecewiseInterval(Range(5.0, true, 10.0, false)) { _ -> 42.0 },
                    PiecewiseInterval(Range(10.0, true, 15.0, false)) { input -> input + 5 },
                )

    private val badRangeExcludingMiddlePoint =
        listOf(
            PiecewiseInterval(Range(0.0, true, 5.0, false)) { input -> input * 2 },
            PiecewiseInterval(Range(5.0, false, 10.0, false)) { _ -> 0.0 },
            PiecewiseInterval(Range(10.0, true, 15.0, false)) { input -> input + 5 },
        )


    @Test
    fun testRangeVerify() {
        val goodPiecewise = RangedPiecewise(testingRange, goodRange)
        val badPiecewise = RangedPiecewise(testingRange, badRangeExcludingMiddlePoint)
        assertTrue("Check for valid range") { goodPiecewise.verifyRange() }
        assertFalse("Check for invalid range") { badPiecewise.verifyRange() }
        assertEquals("Check for inclusive extreme", goodPiecewise.calculate(0.0), 0.0)
        assertEquals("Check for inclusive and exclusive overlap", goodPiecewise.calculate(5.0), 42.0)
        assertEquals("Check for middle of interval", goodPiecewise.calculate(12.0), 12.0 + 5)
        assertFails("Check to ensure calculate() isn't run on invalid piecewise") { badPiecewise.calculate(3.0) }

    }
}