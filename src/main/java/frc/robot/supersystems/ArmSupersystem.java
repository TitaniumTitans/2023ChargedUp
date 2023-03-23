package frc.robot.supersystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
            new PiecewiseInterval<>(stowRange, ignored -> LimitConstants.STOW_LIMIT),
            new PiecewiseInterval<>(fullRangeZone, ignored -> LimitConstants.FULL_RANGE_ZONE),
            /**
             * Extension upper not constant?
             */
            new PiecewiseInterval<>(floorCollisionZone, ignored -> LimitConstants.GROUND_LIMIT)


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
        setToPose(pose, false);
    }
    public void setToPose(ArmPose pose, boolean waitForAngle) {
        ArmLimits armLimits = limitPiecewise.calculate(angleArmSubsystem.getArmAngle());
        double wristSetpoint = pose.wristSetpoint;
        double angleSetpoint = pose.angleSetpoint;
        double extSetpoint = pose.extSetpoint;

        /* Wrist limits and output are calculated here */
        wristSetpoint = armLimits.wristRange.clamp(wristSetpoint);
        wristSubsystem.setWristAngle(wristSetpoint);

        /* Extension limits and outputs are calculated here */
        extSetpoint = armLimits.armExtRange.clamp(extSetpoint);
        extArmSubsystem.setArmExtension(extSetpoint);

        /* Angle limits and outputs are calculated here */
        angleSetpoint = armLimits.armAngleRange.clamp(angleSetpoint);
        angleArmSubsystem.setArmAngle(angleSetpoint);
    }

    public void toggleAllBrakemode() {
        angleArmSubsystem.toggleBrakeMode();
        wristSubsystem.toggleBrakeMode();
        extArmSubsystem.toggleBrakeMode();
    }

    public boolean atSetpoint() {
        return wristSubsystem.atSetpoint() && extArmSubsystem.atSetpoint() && angleArmSubsystem.atSetpoint();
    }

    public CommandBase runIntakeForTime(double seconds, double speed) {
        return wristSubsystem.setIntakeSpeedFactory(speed)
                .andThen(new WaitCommand(seconds))
                .andThen(wristSubsystem.setIntakeSpeedFactory(0));
    }

    public Command runIntake(double speed) {
        return wristSubsystem.setIntakeSpeedFactory(speed);
    }

}
