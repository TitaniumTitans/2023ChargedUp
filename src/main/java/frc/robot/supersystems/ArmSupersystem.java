package frc.robot.supersystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LimitConstants;
import frc.robot.subsystems.arm.ArmAngleSubsystem;
import frc.robot.subsystems.arm.ArmExtSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import lib.utils.piecewise.PiecewiseInterval;
import lib.utils.piecewise.Range;
import lib.utils.piecewise.RangedPiecewise;

import java.util.List;

public class ArmSupersystem {
    private static final Range fullArmRange = new Range(LimitConstants.STOW_ZONE.getValue(), true, LimitConstants.MAX_MOVEMENT.getValue(), false);
    private static final Range stowRange = new Range(LimitConstants.STOW_ZONE.getValue(), true, LimitConstants.SCORE_ZONE.getValue(), false);
    private static final Range fullRangeZone = new Range(LimitConstants.SCORE_ZONE.getValue(), true, LimitConstants.GROUND_ZONE.getValue(), false);
    private static final Range floorCollisionZone = new Range(LimitConstants.GROUND_ZONE.getValue(), true, LimitConstants.MAX_MOVEMENT.getValue(), true);

    private static final List<PiecewiseInterval<ArmLimits>> armRegion = List.of(
            new PiecewiseInterval<>(stowRange, ignored -> new ArmLimits(LimitConstants.WRIST_STOW.getValue(), LimitConstants.WRIST_STOW.getValue(),
                                                                        LimitConstants.ARM_EXT_STOW.getValue(), LimitConstants.ARM_EXT_STOW.getValue(),
                                                                        LimitConstants.ARM_ANGLE_LOWER.getValue(), LimitConstants.ARM_ANGLE_UPPER.getValue())),
            new PiecewiseInterval<>(fullRangeZone, ignored -> new ArmLimits(LimitConstants.WRIST_SCORE_LOWER.getValue(), LimitConstants.WRIST_SCORE_UPPER.getValue(),
                                                                            LimitConstants.ARM_EXT_SCORE_LOWER.getValue(),LimitConstants.ARM_EXT_SCORE_UPPER.getValue(),
                                                                            LimitConstants.ARM_ANGLE_LOWER.getValue(), LimitConstants.ARM_ANGLE_UPPER.getValue())),
            /**
             * Extension upper not constant?
             */
            new PiecewiseInterval<>(floorCollisionZone, ignored -> new ArmLimits(LimitConstants.WRIST_STOW.getValue(), LimitConstants.WRIST_SCORE_UPPER.getValue(),
                                                                                LimitConstants.ARM_EXT_STOW.getValue(), 6.1,
                                                                                LimitConstants.ARM_ANGLE_LOWER.getValue(), LimitConstants.ARM_ANGLE_UPPER.getValue()))
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

    /**
     * Set the arm supersystem to a desired pose as stored in a
     * @param pose An ArmPose object storing the desired pose
     */
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

    public boolean atSetpoint() {
        SmartDashboard.putBoolean("Wrist at setpoint", wristSubsystem.wristAtSetpoint());
        SmartDashboard.putBoolean("EXT at setpoint", extArmSubsystem.armExtensionAtSetpoint());
        SmartDashboard.putBoolean("Angle at setpoint", angleArmSubsystem.armAngleAtSetpoint());
        return (wristSubsystem.wristAtSetpoint() && angleArmSubsystem.armAngleAtSetpoint()) && extArmSubsystem.armExtensionAtSetpoint();
    }
}
