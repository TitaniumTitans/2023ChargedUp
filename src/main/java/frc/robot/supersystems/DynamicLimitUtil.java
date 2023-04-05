package frc.robot.supersystems;

import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class DynamicLimitUtil {
    public double calcExtLimit(double angle) {
        return Math.sin(angle) * ArmConstants.PIVOT_HEIGHT;
    }

    public double calcWristLimit(double angle) {
        return Math.asin(Constants.WristConstants.GROUND_OFFSET);
    }
}
