package frc.robot.supersystems;

import frc.robot.Constants;
import lib.utils.piecewise.Range;

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
    public ArmLimits(double wristLower, double wristUpper,
                     double extLower, double extUpper,
                     double angleLower, double angleUpper) {
        wristRange = new Range(wristLower, true, wristUpper, true);
        armExtRange = new Range(extLower, true, extUpper, true);
        armAngleRange = new Range(angleLower, true, angleUpper, true);
    }

    public boolean withinWristLimit(double inside) {
        return (
                (wristRange.getLeft() > inside || (wristRange.getLeftInclusive() && wristRange.getLeft() == inside)) ||
                (wristRange.getRight() < inside || (wristRange.getRightInclusive() && wristRange.getRight() == inside)));
    }

    public boolean withinExtLimit(double inside) {
        return (
                (armExtRange.getLeft() > inside || (armExtRange.getLeftInclusive() && armExtRange.getLeft() == inside)) ||
                (armExtRange.getRight() < inside || (armExtRange.getRightInclusive() && armExtRange.getRight() == inside)));
    }

    public boolean withinAngleLimit(double inside) {
        return (
                (armAngleRange.getLeft() > inside || (armAngleRange.getLeftInclusive() && armAngleRange.getLeft() == inside)) ||
                (armAngleRange.getRight() < inside || (armAngleRange.getRightInclusive() && armAngleRange.getRight() == inside)));
    }
}
