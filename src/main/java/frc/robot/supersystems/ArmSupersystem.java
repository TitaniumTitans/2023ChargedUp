package frc.robot.supersystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shim.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.LimitConstants;
import frc.robot.subsystems.arm.ArmAngleSubsystem;
import frc.robot.subsystems.arm.ArmExtSubsystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.wrist.WristSubsystem;
import lib.utils.piecewise.PiecewiseInterval;
import lib.utils.piecewise.Range;
import lib.utils.piecewise.RangedPiecewise;

import java.util.List;

public class ArmSupersystem {
    private static final Range fullArmRange = new Range(LimitConstants.STOW_ZONE.getValue(), true, LimitConstants.MAX_MOVEMENT.getValue(), false);
    private static final Range stowRange = new Range(LimitConstants.STOW_ZONE.getValue(), true, LimitConstants.INTAKE_ZONE.getValue(), false);
    private static final Range intakeRange = new Range(LimitConstants.INTAKE_ZONE.getValue(), true, LimitConstants.INTAKE_ZONE_UPPER.getValue(), false);
    private static final Range highStowRange = new Range(LimitConstants.INTAKE_ZONE_UPPER.getValue(), true, LimitConstants.SCORE_ZONE.getValue(), false);
    private static final Range fullRangeZone = new Range(LimitConstants.SCORE_ZONE.getValue(), true, LimitConstants.GROUND_ZONE.getValue(), false);
    private static final Range floorCollisionZone = new Range(LimitConstants.GROUND_ZONE.getValue(), true, LimitConstants.MAX_MOVEMENT.getValue(), true);

    private static final List<PiecewiseInterval<ArmLimits>> armRegion = List.of(
            new PiecewiseInterval<>(stowRange, ignored -> LimitConstants.STOW_LIMIT),
            new PiecewiseInterval<>(fullRangeZone, ignored -> LimitConstants.FULL_RANGE_LIMIT),
            new PiecewiseInterval<>(intakeRange, ignored -> LimitConstants.INTAKE_LIMIT),
            new PiecewiseInterval<>(highStowRange, ignored -> LimitConstants.NO_EXTENTION),
            /*
             * Extension upper not constant?
             */
            new PiecewiseInterval<>(floorCollisionZone, ignored -> LimitConstants.GROUND_LIMIT)
    );

    private static final RangedPiecewise<ArmLimits> limitPiecewise = new RangedPiecewise<>(fullArmRange, armRegion);
    private final ArmAngleSubsystem m_angle;
    private final ArmExtSubsystem m_ext;
    private final WristSubsystem m_wrist;
    private final SwerveDrivetrain m_swerve;

    public ArmSupersystem(ArmAngleSubsystem armSubsystem, ArmExtSubsystem m_ext, WristSubsystem m_wrist, SwerveDrivetrain m_swerve) {
        this.m_angle = armSubsystem;
        this.m_ext = m_ext;
        this.m_wrist = m_wrist;
        this.m_swerve = m_swerve;
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
        ArmLimits armLimits = limitPiecewise.calculate(m_angle.getArmAngle());
        double wristSetpoint = armLimits.wristRange.clamp(pose.getWristSetpoint());
        double angleSetpoint = armLimits.armAngleRange.clamp(pose.getAngleSetpoint());
        double extSetpoint = armLimits.armExtRange.clamp(pose.extSetpoint);

        // Wrist limits and output are calculated here
        m_wrist.setWristAngle(wristSetpoint);

        // Extension limits and outputs are calculated here
        if (extSetpoint < 5 || Math.abs(m_angle.getError()) < 15) {
            m_ext.setArmExtension(extSetpoint);
        }

        // Angle limits and outputs are calculated here
        m_angle.setArmAngle(calculateArmAngleLimit(angleSetpoint));
    }

    public double calculateArmAngleLimit(double desiredAngle) {
        // Angle limits and outputs are calculated here
        double currentArmAngle = m_angle.getArmAngle();
        double currentArmExtension = m_ext.getArmExtension();
        double currentWristAngle = m_wrist.getWristAngle();

        // Added safety when on the battery side of the bot
        if((currentArmAngle < 75)  && (currentWristAngle > 10 || currentArmExtension > 0.6)) {
            Translation2d physicalArmExtension = getPhysicalArmExtension();
            // Limit the angle to 50 degrees or higher while the arm is extended and wrist out
            /*
                      / |
 arm ext = 36" + ext /  |
                 ---/   |   40in
             wrist^     |
              -----------  Robot base

             */
            final double armPhysicalExtensionNormal = physicalArmExtension.getNorm();
            double angleLimit;
            if(armPhysicalExtensionNormal > Constants.ArmConstants.PIVOT_HEIGHT) {
                double angleAdjust = currentArmAngle - physicalArmExtension.getAngle().getDegrees();
                angleLimit = Math.max(48, Math.toDegrees(Math.acos(Constants.ArmConstants.PIVOT_HEIGHT / physicalArmExtension.getNorm())) + angleAdjust + 2);
                SmartDashboard.putNumber("SuperAngleAdjust", angleAdjust);
            } else {
                angleLimit = 50;
            }
            SmartDashboard.putNumber("SuperAngleLimit", angleLimit);
            desiredAngle = Math.max(desiredAngle, angleLimit);
            SmartDashboard.putNumber("SuperCalcAngle", physicalArmExtension.getAngle().getDegrees());
            SmartDashboard.putNumber("SuperCalcExt", physicalArmExtension.getNorm());
        }

        SmartDashboard.putNumber("Previous Arm Angle Limit", desiredAngle);
        SmartDashboard.putNumber("SuperArmAngle", currentArmAngle);
        SmartDashboard.putNumber("SuperArmExt", currentArmExtension);
        SmartDashboard.putNumber("SuperWristAngle", currentWristAngle);

        return desiredAngle;
    }

    public void toggleAllBrakemode() {
        m_angle.toggleBrakeMode();
        m_wrist.toggleBrakeMode();
        m_ext.toggleBrakeMode();
    }

    public void enableAllBrakemode() {
        m_angle.setBrakeMode(CANSparkMax.IdleMode.kBrake);
        m_wrist.setBrakeMode(CANSparkMax.IdleMode.kBrake);
        m_ext.setBrakeMode(CANSparkMax.IdleMode.kBrake);
    }

    public void disableAllBrakemode() {
        m_angle.setBrakeMode(CANSparkMax.IdleMode.kCoast);
        m_wrist.setBrakeMode(CANSparkMax.IdleMode.kCoast);
        m_ext.setBrakeMode(CANSparkMax.IdleMode.kCoast);
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

    public Translation2d getPhysicalArmExtension() {
        double currentArmAngleRadians = Math.toRadians(m_angle.getArmAngle());
        double currentWristAngleRadians = Math.toRadians(m_wrist.getWristAngle());
        double physicalExtension = m_ext.getPhysicalExtension();
        double physicalX = Math.cos(currentArmAngleRadians) * physicalExtension;
        double physicalY = Math.sin(currentArmAngleRadians) * physicalExtension;
        Translation2d physicalVector = new Vector2d(physicalX, physicalY);
        double wristRelativeAngle = currentWristAngleRadians - currentArmAngleRadians;
        // Estimate the wrist extension into the ground
        if (wristRelativeAngle > 0) {
            final double physicalWristExt = 10;
            return physicalVector.plus(new Translation2d(physicalWristExt * Math.sin(wristRelativeAngle), physicalWristExt * Math.cos(wristRelativeAngle)));
        }
        return physicalVector;
    }

    public double clampArmExt (double setpoint, double startZone) {
    return setpoint * (Math.pow(0.1, Constants.ArmConstants.EXT_AGRESSION))
                * Math.pow(MathUtil.clamp(Math.abs(m_angle.getError()), 0.0, startZone) - startZone,
                Constants.ArmConstants.EXT_AGRESSION);
    }

    public double clampArmAngle () {
        return Constants.ArmConstants.MAX_ANGLE_SPEED
                * Math.pow((m_ext.getArmExtension()/Constants.ArmConstants.ANGLE_AGRESSION) - 1,
                Constants.ArmConstants.ANGLE_AGRESSION);
    }

    public void getDriveSpeed () {
        Double range = LimitConstants.DRIVE_SPEED_PIECEWISE.calculate(m_angle.getArmAngle());
        m_swerve.setSpeedMult(range.doubleValue());

        if (m_ext.getArmExtension() > 5) {
            m_swerve.setRotationMult(0.5);
        } else {
            m_swerve.setRotationMult(1);
        }
    }

}
