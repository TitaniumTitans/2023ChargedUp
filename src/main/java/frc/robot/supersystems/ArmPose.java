package frc.robot.supersystems;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class ArmPose implements Sendable {
    public final double extSetpoint;
    public final DoubleSupplier angleSetpointSupplier;
    public final DoubleSupplier wristSetpoint;

    public ArmPose(double ext, double angle, double wrist) {
        this(ext, () -> angle, () -> wrist);
    }

    public ArmPose(double ext, DoubleSupplier angleSupplier, DoubleSupplier wrist) {
        if (ext < 0.0 || angleSupplier.getAsDouble() < 0.0 || wrist.getAsDouble() < 0.0) {
            throw new IllegalStateException("Not a valid ArmPose parameter");
        }
        extSetpoint = ext;
        angleSetpointSupplier = angleSupplier;
        wristSetpoint = wrist;
    }

    public double getAngleSetpoint() {
        return angleSetpointSupplier.getAsDouble();
    }

    public double getWristSetpoint() {
        return wristSetpoint.getAsDouble();
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
