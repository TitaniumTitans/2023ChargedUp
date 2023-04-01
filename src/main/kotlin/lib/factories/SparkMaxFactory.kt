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
        var frame0Rate: Int = 100,
        var frame1Rate: Int = 200,
        var frame2Rate: Int = 40,
        var frame3Rate: Int = MAX_CAN_FRAME_PERIOD,
        var frame4Rate: Int = MAX_CAN_FRAME_PERIOD,
        var frame5Rate: Int = MAX_CAN_FRAME_PERIOD,
        var frame6Rate: Int = MAX_CAN_FRAME_PERIOD,
        var idleMode: CANSparkMax.IdleMode = CANSparkMax.IdleMode.kBrake,
        var inverted: Boolean = false,
        var currentLimit: Int = 30,
        var followingMotor: CANSparkMax? = null
    )

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
    data class SparkWithConfig(val spark: CANSparkMax, val config: SparkMaxConfig = SparkMaxConfig())

    companion object {
        const val MAX_CAN_FRAME_PERIOD = 65535

        // A list of all configured sparks linked to thier configs
        val listOfAllSparksAndConfigs: MutableList<SparkWithConfig> = ArrayList()
        fun updateCanFramePeriods() {
            for (sparkAndConfig in listOfAllSparksAndConfigs) {
                val spark = sparkAndConfig.spark
                val config = sparkAndConfig.config
                if (spark.getStickyFault(CANSparkMax.FaultID.kHasReset)) {
                    RevUtil.autoRetry { spark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, config.frame0Rate) }
                    RevUtil.autoRetry { spark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, config.frame1Rate) }
                    RevUtil.autoRetry { spark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, config.frame2Rate) }
                    RevUtil.autoRetry { spark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, config.frame3Rate) }
                    RevUtil.autoRetry { spark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus4, config.frame4Rate) }
                    RevUtil.autoRetry { spark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus5, config.frame5Rate) }
                    RevUtil.autoRetry { spark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus6, config.frame6Rate) }
                }
            }
        }

        fun rerunConfigs() {
            for (sparkAndConfig in listOfAllSparksAndConfigs) {
                val spark = sparkAndConfig.spark
                val config = sparkAndConfig.config

                RevUtil.autoRetry { spark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, config.frame0Rate) }
                RevUtil.autoRetry { spark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, config.frame1Rate) }
                RevUtil.autoRetry { spark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, config.frame2Rate) }
                RevUtil.autoRetry { spark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, config.frame3Rate) }
                RevUtil.autoRetry { spark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus4, config.frame4Rate) }
                RevUtil.autoRetry { spark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus5, config.frame5Rate) }
                RevUtil.autoRetry { spark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus6, config.frame6Rate) }

                RevUtil.autoRetry { spark.setIdleMode(config.idleMode) }
                spark.inverted = config.inverted
                RevUtil.autoRetry { spark.setSmartCurrentLimit(config.currentLimit) }
            }
        }

        /**
         * Returns a Spark Max motor controller set to the config handed to it
         * @return A configured spark max motor controller
         */

        fun createSparkMax(id: Int, config: SparkMaxConfig): CANSparkMax {
            val spark = CANSparkMax(id, CANSparkMaxLowLevel.MotorType.kBrushless)

            RevUtil.autoRetry { spark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, config.frame0Rate) }
            RevUtil.autoRetry { spark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, config.frame1Rate) }
            RevUtil.autoRetry { spark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, config.frame2Rate) }
            RevUtil.autoRetry { spark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, config.frame3Rate) }
            RevUtil.autoRetry { spark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus4, config.frame4Rate) }
            RevUtil.autoRetry { spark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus5, config.frame5Rate) }
            RevUtil.autoRetry { spark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus6, config.frame6Rate) }

            RevUtil.autoRetry { spark.setIdleMode(config.idleMode) }
            if(config.followingMotor != null) {
                RevUtil.autoRetry { spark.follow(config.followingMotor, config.inverted) }
            } else {
                spark.inverted = config.inverted
            }
            RevUtil.autoRetry { spark.setSmartCurrentLimit(config.currentLimit) }

            listOfAllSparksAndConfigs.add(SparkWithConfig(spark, config.copy()))

            spark.burnFlash()

            return spark
        }
    }
}
