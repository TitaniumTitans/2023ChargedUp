package lib.factories

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel

class SparkMaxFactory {
    /**
     * A data class storing the configuration values of a spark max motor controller
     * @param frame0Rate the periodic frame of status 0 in milliseconds
     * @param frame1Rate the periodic frame of status 1 in milliseconds
     * @param frame2Rate the periodic frame of status 2 in milliseconds
     * @param frame3Rate the periodic frame of status 3 in milliseconds
     * @param frame4Rate the periodic frame of status 4 in milliseconds
     * @param frame5Rate the periodic frame of status 5 in milliseconds
     * @param frame6Rate the periodic frame of status 6 in milliseconds
     * @param idleMode the idle mode of the motor
     * @param inverted the invertion of the motor
     * @param currentLimit the max current draw allowed for the motor controller
     */
    data class SparkMaxConfig(
            val frame0Rate: Int,
            val frame1Rate: Int,
            val frame2Rate: Int,
            val frame3Rate: Int,
            val frame4Rate: Int,
            val frame5Rate: Int,
            val frame6Rate: Int,
            val idleMode: CANSparkMax.IdleMode,
            val inverted: Boolean,
            val currentLimit: Int)

    val defaultConfig: SparkMaxConfig = SparkMaxConfig(
            200,
            200,
            200,
            200,
            200,
            200,
            200,
            CANSparkMax.IdleMode.kBrake,
            false,
            20
    )

    /**
     * Returns a Spark Max motor controller set to the config handed to it
     * @return A configured spark max motor controller
     */
    fun createSparkMax(id: Int, config: SparkMaxConfig): CANSparkMax {
        var spark: CANSparkMax = CANSparkMax(id, CANSparkMaxLowLevel.MotorType.kBrushless)
        spark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, config.frame0Rate)
        spark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, config.frame1Rate)
        spark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, config.frame2Rate)
        spark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, config.frame3Rate)
        spark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus4, config.frame4Rate)
        spark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus5, config.frame5Rate)
        spark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus6, config.frame6Rate)

        spark.setIdleMode(config.idleMode)
        spark.setInverted(config.inverted)
        spark.setSmartCurrentLimit(config.currentLimit)

        return spark
    }

    /**
     * Creates a Spark Max to these settings:
     *  frame rate 0-6: 200 milliseconds
     *  idle mode: brake
     *  inverted: false
     *  current limit: 20 amps
     * @return a spark max configured to the default settings
     */
    fun createDefaultSpark(id: Int): CANSparkMax {
        return createSparkMax(id, defaultConfig)
    }
}