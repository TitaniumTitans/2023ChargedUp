package lib.utils.Rev;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

public class SparkMaxConfigs {
    private static final int FAST_CAN_FRAME = 10;
    private static final int MEDIUM_CAN_FRAME = 100;
    private static final int SLOW_CAN_FRAME = 200;
    private static final int DISABLED_CAN_FRAME = 65535;

    public static void configCanStatusFrames (CANSparkMax spark){
        spark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, MEDIUM_CAN_FRAME);
        spark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, FAST_CAN_FRAME);
        spark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, FAST_CAN_FRAME);
        spark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, SLOW_CAN_FRAME);
        spark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus4, DISABLED_CAN_FRAME);
        spark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus5, FAST_CAN_FRAME);
        spark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus6, SLOW_CAN_FRAME);
    }
}
