package frc.robot.supersystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmAngleSubsystem;
import frc.robot.subsystems.arm.ArmExtSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import lib.utils.piecewise.PiecewiseInterval;
import lib.utils.piecewise.Range;
import lib.utils.piecewise.RangedPiecewise;

import java.util.List;

public class ArmSupersystem {
    private static final Range fullArmRange = new Range(30, true, 300, false);
    private static final Range stowRange = new Range(30, true, 200, false);
    private static final Range fullRangeZone = new Range(200, true, 280, false);
    private static final Range floorCollisionZone = new Range(280, true, 300, false);

    //TODO before testing implement actual limits in each zone
    private static final List<PiecewiseInterval<ArmLimits>> armRegion = List.of(
            new PiecewiseInterval<>(stowRange, ignored -> new ArmLimits(0.0, 0.0,
                                                                        0.0, 0.0,
                                                                        0.0, 0.0)),
            new PiecewiseInterval<>(fullRangeZone, ignored -> new ArmLimits(0.0, 0.0,
                                                                            0.0, 0.0,
                                                                            0.0, 0.0)),
            new PiecewiseInterval<>(floorCollisionZone, ignored -> new ArmLimits(0.0, 0.0,
                                                                                0.0, 0.0,
                                                                                0.0, 0.0))
    );
    private static final RangedPiecewise<ArmLimits> limitPiecewise = new RangedPiecewise<>(fullArmRange, armRegion);
    private final ArmAngleSubsystem angleArmSubsystem;
    private final ArmExtSubsystem extArmSubsystem;
    private final WristSubsystem wristSubsystem;

    public ArmSupersystem(ArmAngleSubsystem armSubsystem, ArmExtSubsystem extArmSubsystem, WristSubsystem wristSubsystem) {
        this.angleArmSubsystem = armSubsystem;
        this.extArmSubsystem = extArmSubsystem;
        this.wristSubsystem = wristSubsystem;
    }

    public void addRequirements(CommandBase command) {
        command.addRequirements(wristSubsystem, angleArmSubsystem, extArmSubsystem);
    }

    public void stopSpeed() {
        wristSubsystem.setWristPower(0.0);
        extArmSubsystem.setArmSpeed(0.0);
        angleArmSubsystem.setAngleSpeed(0.0);
    }

    public void setToPose(ArmPose pose) {
        ArmLimits armLimits = limitPiecewise.calculate(angleArmSubsystem.getArmAngle());
        double wristSetpoint = pose.wristSetpoint;
        double angleSetpoint = pose.angleSetpoint;
        double extSetpoint = pose.extSetpoint;

        /** Wrist limits and output are calculated here */
        wristSetpoint = MathUtil.clamp(wristSetpoint, armLimits.wristRange.getLeft(), armLimits.wristRange.getRight());
        wristSubsystem.setWristAngle(wristSetpoint);

        /** Extension limits and outputs are calculated here */
        extSetpoint = MathUtil.clamp(extSetpoint, armLimits.armExtRange.getLeft(), armLimits.armExtRange.getRight());
        extArmSubsystem.setArmExtension(extSetpoint);

        /** Angle limits and outputs are calculated here */
        angleSetpoint = MathUtil.clamp(angleSetpoint, armLimits.armAngleRange.getLeft(), armLimits.armAngleRange.getRight());
        angleArmSubsystem.setArmAngle(angleSetpoint);
    }
}
