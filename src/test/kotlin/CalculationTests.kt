import org.junit.jupiter.api.Test
import kotlin.math.asin
import kotlin.math.sin
import kotlin.test.assertEquals

class CalculationTests {
    @Test
    fun testBasicCalc() {
        // Test to see if we can calculate an angle based on its sine value
        assertEquals(1.0, asin(sin((1.0))), "Test inverse sine function")
    }
}