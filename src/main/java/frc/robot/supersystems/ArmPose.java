package frc.robot.supersystems;

public class ArmPose {
    public final double extSetpoint;
    public final double angleSetpoint;
    public final double wristSetpoint;

    public ArmPose(double ext, double angle, double wrist) {
        if (ext < 0.0 || angle < 0.0 || wrist < 0.0) {
            throw new RuntimeException("Not a valid ArmPose parameter");
        }
        extSetpoint = ext;
        angleSetpoint = angle;
        wristSetpoint = wrist;
    }
}
