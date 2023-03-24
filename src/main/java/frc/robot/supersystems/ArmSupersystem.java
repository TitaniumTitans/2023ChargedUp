package frc.robot.supersystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
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
    private static final Range stowRange = new Range(LimitConstants.STOW_ZONE.getValue(), true, LimitConstants.INTAKE_ZONE.getValue(), false);
    private static final Range intakeRange = new Range(LimitConstants.INTAKE_ZONE.getValue(), true, LimitConstants.SCORE_ZONE.getValue(), false);
    private static final Range fullRangeZone = new Range(LimitConstants.SCORE_ZONE.getValue(), true, LimitConstants.GROUND_ZONE.getValue(), false);
    private static final Range floorCollisionZone = new Range(LimitConstants.GROUND_ZONE.getValue(), true, LimitConstants.MAX_MOVEMENT.getValue(), true);

    private static final List<PiecewiseInterval<ArmLimits>> armRegion = List.of(
            new PiecewiseInterval<>(stowRange, ignored -> LimitConstants.STOW_LIMIT),
            new PiecewiseInterval<>(fullRangeZone, ignored -> LimitConstants.FULL_RANGE_LIMIT),
            new PiecewiseInterval<>(intakeRange, ignored -> LimitConstants.INTAKE_LIMIT),
            /**
             * Extension upper not constant?
             */
            new PiecewiseInterval<>(floorCollisionZone, ignored -> LimitConstants.GROUND_LIMIT)


    );
    private static final RangedPiecewise<ArmLimits> limitPiecewise = new RangedPiecewise<>(fullArmRange, armRegion);
    private final ArmAngleSubsystem m_angle;
    private final ArmExtSubsystem m_ext;
    private final WristSubsystem m_wrist;

    public ArmSupersystem(ArmAngleSubsystem armSubsystem, ArmExtSubsystem m_ext, WristSubsystem m_wrist) {
        this.m_angle = armSubsystem;
        this.m_ext = m_ext;
        this.m_wrist = m_wrist;
    }

    public void addRequirements(CommandBase command) {
        command.addRequirements(m_wrist, m_angle, m_ext);
    }

    public void stopSpeed() {
        m_wrist.setWristPower(0.0);
        m_ext.setArmSpeed(0.0);
        m_angle.setAngleSpeed(0.0);
    }

    /**
     * Set the arm supersystem to a desired pose as stored in a
     * @param pose An ArmPose object storing the desired pose
     */
    public void setToPose(ArmPose pose) {
        setToPose(pose, false);
    }
    public void setToPose(ArmPose pose, boolean waitForAngle) {
        ArmLimits armLimits = limitPiecewise.calculate(m_angle.getArmAngle());
        double wristSetpoint = pose.wristSetpoint;
        double angleSetpoint = pose.angleSetpoint;
        double extSetpoint = pose.extSetpoint;

        // Wrist limits and output are calculated here
        wristSetpoint = armLimits.wristRange.clamp(wristSetpoint);
        m_wrist.setWristAngle(wristSetpoint);

        // Extension limits and outputs are calculated here
        extSetpoint = armLimits.armExtRange.clamp(extSetpoint);
        m_ext.setArmExtension(extSetpoint);

        // Angle limits and outputs are calculated here
        if (!waitForAngle || (m_ext.atSetpoint() && m_wrist.atSetpoint())) {
            angleSetpoint = armLimits.armAngleRange.clamp(angleSetpoint);
            m_angle.setArmAngle(angleSetpoint);
        } else {
            m_angle.setAngleSpeed(0.0);
        }
    }

    public void toggleAllBrakemode() {
        m_angle.toggleBrakeMode();
        m_wrist.toggleBrakeMode();
        m_ext.toggleBrakeMode();
    }

    public boolean atSetpoint() {
        return m_wrist.atSetpoint() && m_ext.atSetpoint() && m_angle.atSetpoint();
    }

    public CommandBase runIntakeForTime(double seconds, double speed) {
        return m_wrist.setIntakeSpeedFactory(speed)
                .andThen(new WaitCommand(seconds))
                .andThen(m_wrist.setIntakeSpeedFactory(0));
    }

    public Command runIntake(double speed) {
        return m_wrist.setIntakeSpeedFactory(speed);
    }

}
