package lib.factories

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import lib.utils.drivers.RevUtil

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
     * @param inverted the inversion of the motor
     * @param currentLimit the max current draw allowed for the motor controller
     */
    data class SparkMaxConfig(
            val frame0Rate: Int = MAX_CAN_FRAME_PERIOD,
            val frame1Rate: Int = 200,
            val frame2Rate: Int = 20,
            val frame3Rate: Int = MAX_CAN_FRAME_PERIOD,
            val frame4Rate: Int = MAX_CAN_FRAME_PERIOD,
            val frame5Rate: Int = MAX_CAN_FRAME_PERIOD,
            val frame6Rate: Int = MAX_CAN_FRAME_PERIOD,
            val idleMode: CANSparkMax.IdleMode = CANSparkMax.IdleMode.kBrake,
            val inverted: Boolean = false,
            val currentLimit: Int = 30)


    /**
     * Creates a Spark Max to these settings:
     *  frame rate 0, 3-6: 65535 milliseconds
     *  frame rate 1: 200 milliseconds
     *  frame rate 2: 20 milliseconds
     *  idle mode: brake
     *  inverted: false
     *  current limit: 30 amps
     * @return a spark max configured to the default settings
     */
    fun createDefaultSpark(id: Int): CANSparkMax {
        return Companion.createSparkMax(id, SparkMaxConfig())
    }

    companion object {
       const val MAX_CAN_FRAME_PERIOD = 65535;

        /**
         * Returns a Spark Max motor controller set to the config handed to it
         * @return A configured spark max motor controller
         */
        fun createSparkMax(id: Int, config: SparkMaxConfig): CANSparkMax {
            val spark = CANSparkMax(id, CANSparkMaxLowLevel.MotorType.kBrushless)
            RevUtil.autoRetry {spark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, config.frame0Rate)}
            RevUtil.autoRetry {spark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, config.frame1Rate)}
            RevUtil.autoRetry {spark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, config.frame2Rate)}
            RevUtil.autoRetry {spark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, config.frame3Rate)}
            RevUtil.autoRetry {spark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus4, config.frame4Rate)}
            RevUtil.autoRetry {spark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus5, config.frame5Rate)}
            RevUtil.autoRetry {spark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus6, config.frame6Rate)}

            RevUtil.autoRetry {spark.setIdleMode(config.idleMode)}
            spark.inverted = config.inverted
            RevUtil.autoRetry {spark.setSmartCurrentLimit(config.currentLimit)}

            return spark
        }
    }
}