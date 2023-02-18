package frc.robot.supersystems;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

import java.util.function.DoubleConsumer;

public class ArmPose implements Sendable {
    public final double extSetpoint;
    public final double angleSetpoint;
    public final double wristSetpoint;

    public ArmPose(double ext, double angle, double wrist) {
        if (ext < 0.0 || angle < 0.0 || wrist < 0.0) {
            throw new IllegalStateException("Not a valid ArmPose parameter");
        }
        extSetpoint = ext;
        angleSetpoint = angle;
        wristSetpoint = wrist;
    }

    public double getAngleSetpoint() {
        return angleSetpoint;
    }

    public double getWristSetpoint() {
        return wristSetpoint;
    }

    public double getExtSetpoint() {
        return extSetpoint;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        DoubleConsumer nothingSetter = ignored -> {};
        builder.addDoubleProperty("Angle", this::getAngleSetpoint, nothingSetter);
        builder.addDoubleProperty("Extension", this::getExtSetpoint, nothingSetter);
        builder.addDoubleProperty("Wrist Angle", this::getWristSetpoint, nothingSetter);
    }
}
