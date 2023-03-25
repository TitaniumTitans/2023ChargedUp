package frc.robot.supersystems;

import lib.utils.piecewise.Range;
import java.util.Objects;

public class ArmLimits {
    public final Range armExtRange;
    public final Range armAngleRange;
    public final Range wristRange;

    /**
     * creates a new Arm Limit class from double inputs
     * @param wristLower wrist lower limit
     * @param wristUpper wrist upper limit
     * @param extLower extension lower limit
     * @param extUpper extension upper limit
     * @param angleLower angle lower limit
     * @param angleUpper angle upper limit
     */
    public ArmLimits(double wristLower, double wristUpper, double extLower, double extUpper,
                     double angleLower, double angleUpper) {
        wristRange = new Range(wristLower, true, wristUpper, true);
        armExtRange = new Range(extLower, true, extUpper, true);
        armAngleRange = new Range(angleLower, true, angleUpper, true);
    }

    public boolean withinWristLimit(double inside) {
        return wristRange.withinRange(inside);
    }

    public boolean withinExtLimit(double inside) {
        return armExtRange.withinRange(inside);
    }

    public boolean withinAngleLimit(double inside) {
        return armAngleRange.withinRange(inside);
    }
    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        ArmLimits armLimits = (ArmLimits) o;
        return armExtRange.equals(armLimits.armExtRange) && armAngleRange.equals(armLimits.armAngleRange) && wristRange.equals(armLimits.wristRange);
    }

    @Override
    public int hashCode() {
        return Objects.hash(armExtRange, armAngleRange, wristRange);
    }
}
