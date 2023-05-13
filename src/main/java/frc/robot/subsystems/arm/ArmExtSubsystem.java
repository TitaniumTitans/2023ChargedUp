package frc.robot.subsystems.arm;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lib.factories.SparkMaxFactory;
import lib.utils.drivers.RevUtil;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class ArmExtSubsystem extends SubsystemBase {
    private final CANSparkMax m_armExt;
    private final RelativeEncoder m_relativeEncoderArmEx;
    private final SparkMaxPIDController m_extPID;
//    private final PIDController m_extPID;
    private final DigitalInput m_armLimitSwitch;

    private final ArmExtIOInputsAutoLogged m_inputs = new ArmExtIOInputsAutoLogged();
    // Logging variables
    private double prevSetpointRaw;
    private double prevSetpointClamped;
    public static boolean m_hasArmHomed = false;
    //Shuffleboard data
    private final ShuffleboardTab armExtTab;
    private GenericEntry armExtAtSetpointEntry;
    private GenericEntry armExtAtUpperLimitEntry;
    private GenericEntry armExtAtLowerLimitEntry;
    private GenericEntry armExtConvertedEntry;
    private GenericEntry armExtTargetEntry;
    private GenericEntry armExtSetpointClampedEntry;
    private GenericEntry armExtMotorOutputEntry;





    @AutoLog
    public static class ArmExtIOInputs {
        public double armExtension = 0.0;
    }

    public ArmExtSubsystem() {
        SparkMaxFactory.SparkMaxConfig config = new SparkMaxFactory.SparkMaxConfig();
        config.setCurrentLimit(50);
        config.setInverted(Constants.CURRENT_MODE == Constants.Mode.HELIOS_V1);

        m_armExt = SparkMaxFactory.Companion.createSparkMax(Constants.ArmConstants.ARM_EXTENSION_ID, config);
        m_armExt.setClosedLoopRampRate(0.1);

        m_relativeEncoderArmEx = m_armExt.getEncoder();
        RevUtil.autoRetry(() -> m_relativeEncoderArmEx.setPositionConversionFactor(Constants.ArmConstants.EXTENSION_RATIO));

        m_extPID = m_armExt.getPIDController();
        m_extPID.setP(Constants.ArmConstants.ARM_EXT_KP.getValue());
        m_extPID.setI(Constants.ArmConstants.ARM_EXT_KI.getValue());
        m_extPID.setD(Constants.ArmConstants.ARM_EXT_KD.getValue());
        m_extPID.setOutputRange(-0.4, 0.4);
        // m_extPID.setOutputRange(-0.75, 0.75);

//        m_extPID = new PIDController(
//                Constants.ArmConstants.ARM_EXT_KP.getValue(),
//                Constants.ArmConstants.ARM_EXT_KI.getValue(),
//                Constants.ArmConstants.ARM_EXT_KD.getValue());
//        m_extPID.setTolerance(0.5);
//        m_extPID.disableContinuousInput();

        m_armLimitSwitch = new DigitalInput(Constants.ArmConstants.LIMIT_SWITCH_PORT);

        armExtTab = Shuffleboard.getTab("ArmExtSubsystem");
        addShuffleboardData();
    }
    private void addShuffleboardData() {
        // Booleans
        // Misc.
        armExtAtSetpointEntry = armExtTab.add("At setpoint", atSetpoint()).getEntry();
        // Limits
        armExtAtUpperLimitEntry = armExtTab.add("At upper limit", armAtUpperLimit()).getEntry();
        armExtAtLowerLimitEntry = armExtTab.add("Limit switch triggered", armAtLowerLimit()).getEntry();

        // Doubles
        // Angles
        armExtConvertedEntry = armExtTab.add("Arm Extension", getArmExtension()).getEntry();
        // Targets
        armExtTargetEntry = armExtTab.add("Target", prevSetpointRaw).getEntry();
        armExtSetpointClampedEntry = armExtTab.add("Clamped setpoint", prevSetpointClamped).getEntry();
        // Misc.
        armExtMotorOutputEntry = armExtTab.add("Motor output", m_armExt.getAppliedOutput()).getEntry();
    }

    private void updateShuffleboardData() {
        // Booleans
        // Misc.
        armExtAtSetpointEntry.setBoolean(atSetpoint());
        // Limits
        armExtAtUpperLimitEntry.setBoolean(armAtUpperLimit());
        armExtAtLowerLimitEntry.setBoolean(armAtLowerLimit());

        // Doubles
        // Angles
        armExtConvertedEntry.setDouble(getArmExtension());
        // Targets
        armExtTargetEntry.setDouble(prevSetpointRaw);
        armExtSetpointClampedEntry.setDouble(prevSetpointClamped);
        // Misc.
        armExtMotorOutputEntry.setDouble(m_armExt.getAppliedOutput());
    }

    public void setArmSpeed(double speed) {
        if((armAtLowerLimit() || getArmExtension() <= Constants.LimitConstants.ARM_EXT_STOW.getValue()) && speed <= 0
                || armAtUpperLimit() && speed >= 0){
            m_armExt.set(0);
        } else {
            m_armExt.set(speed);
        }
    }

    public Command setArmSpeedFactory(double speed) {
        return runOnce(() -> setArmSpeed(speed));
    }

    public void setArmExtension(double targetExtRaw) {
        double targetExtClamped = MathUtil.clamp(targetExtRaw, Constants.LimitConstants.ARM_EXT_SCORE_LOWER.getValue(), Constants.LimitConstants.ARM_EXT_SCORE_UPPER.getValue());

//        double pidOutput = MathUtil.clamp(m_extPID.calculate(m_relativeEncoderArmEx.getPosition(), targetExtClamped), -0.25, 0.25);
        prevSetpointRaw = targetExtRaw;
        prevSetpointClamped = targetExtClamped;

        if(armAtLowerLimit() && targetExtClamped <= 0) {
            m_armExt.set(0);
        } else {
//            setArmSpeed(pidOutput);
            m_extPID.setReference(targetExtClamped, CANSparkMax.ControlType.kPosition);
        }
    }

    public double getArmExtension() {
        return m_relativeEncoderArmEx.getPosition();
    }

    public boolean armAtLowerLimit() {
        return !m_armLimitSwitch.get();
    }

    public boolean armAtUpperLimit() {
        return (getArmExtension() >= Constants.LimitConstants.ARM_EXT_SCORE_UPPER.getValue());
    }

    public boolean debugEncoderAtLowerLimit() {
        return (getArmExtension() <= Constants.LimitConstants.ARM_EXT_SCORE_LOWER.getValue());
    }

    public void resetExtensionEncoder() {
        m_relativeEncoderArmEx.setPosition(0.0);
    }

    public boolean atSetpoint() {
        double armExtension = getArmExtension();
        return (armExtension <= prevSetpointClamped + Constants.ArmConstants.EXT_PID_TOLERANCE)
                && (armExtension >= prevSetpointClamped - Constants.ArmConstants.EXT_PID_TOLERANCE);
    }

    @Override
    public void periodic() {
        updateInputs(m_inputs);
        Logger.getInstance().processInputs("Arm Extension", m_inputs);
        if(armAtLowerLimit())
        {
            resetExtensionEncoder();
        }
        updateShuffleboardData();

        SmartDashboard.putBoolean("Periodic Ext at setpoint", atSetpoint());
    }

    public void toggleBrakeMode() {
        if (m_armExt.getIdleMode() == CANSparkMax.IdleMode.kBrake) {
            m_armExt.setIdleMode(CANSparkMax.IdleMode.kCoast);
        } else {
            m_armExt.setIdleMode(CANSparkMax.IdleMode.kBrake);
        }
    }

    public void updateInputs(ArmExtIOInputsAutoLogged inputs) {
        inputs.armExtension = getArmExtension();
    }

    public boolean hasArmHomed() { return m_hasArmHomed; }

    public void resetHomed() {
        m_hasArmHomed = false;
    }

    public void goArmToHome() {
        if (!hasArmHomed()) {
            if (armAtLowerLimit()) {
                setArmSpeed(0);
                m_hasArmHomed = true;
            } else {
                m_armExt.set(-0.2);
            }
        }
    }

    public double getPhysicalExtension() {
        return getArmExtension() + 24;
    }

    public void setBrakeMode(CANSparkMax.IdleMode brakeMode) {
        m_armExt.setIdleMode(brakeMode);
    }

}

