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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lib.factories.SparkMaxFactory;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class ArmExtSubsystem extends SubsystemBase {
    private final CANSparkMax m_armExt;
    private final RelativeEncoder m_relativeEncoderArmEx;
    private final SparkMaxPIDController m_extPID;
    private final DigitalInput m_armLimitSwitch;

    private final ArmExtIOInputsAutoLogged m_inputs = new ArmExtIOInputsAutoLogged();
    // Logging variables
    private double prevSetpointRaw;
    private double prevSetpointClamped;

    //Shuffleboard data
    private final ShuffleboardTab armExtTab;
    private GenericEntry armExtAtSetpointEntry;
    private GenericEntry armExtMotorInvertedEntry;
    private GenericEntry armExtAtUpperLimitEntry;
    private GenericEntry armExtAtLowerLimitEntry;
    private GenericEntry armExtEncoderLowerThanLimitEntry;
//    private GenericEntry armExtRawEntry;
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

        m_armExt = SparkMaxFactory.Companion.createSparkMax(Constants.ArmConstants.ARM_EXTENSION_ID, config);

        m_relativeEncoderArmEx = m_armExt.getEncoder();
        m_relativeEncoderArmEx.setPositionConversionFactor(Constants.ArmConstants.EXTENSION_RATIO);

        m_extPID = m_armExt.getPIDController();
        m_extPID.setP(Constants.ArmConstants.ARM_EXT_KP.getValue());
        m_extPID.setI(Constants.ArmConstants.ARM_EXT_KI.getValue());
        m_extPID.setD(Constants.ArmConstants.ARM_EXT_KD.getValue());


        m_armLimitSwitch = new DigitalInput(Constants.ArmConstants.LIMIT_SWITCH_PORT);

        armExtTab = Shuffleboard.getTab("ArmExtSubsystem");
        addShuffleboardData();
    }
    private void addShuffleboardData() {
        // Booleans
        // Misc.
        armExtAtSetpointEntry = armExtTab.add("At setpoint", armExtensionAtSetpoint()).getEntry();
        armExtMotorInvertedEntry = armExtTab.add("Motor inverted", m_armExt.getInverted()).getEntry();
        // Limits
        armExtAtUpperLimitEntry = armExtTab.add("At upper limit", armAtUpperLimit()).getEntry();
        armExtAtLowerLimitEntry = armExtTab.add("Limit switch triggered", armAtLowerLimit()).getEntry();
        armExtEncoderLowerThanLimitEntry = armExtTab.add("Wrist encoder lower than limit", debugEncoderAtLowerLimit()).getEntry();

        // Doubles
        // Angles
//        armExtRawEntry = armExtTab.add("Angle raw", m_relativeEncoderArmEx.getPosition()).getEntry();
        armExtConvertedEntry = armExtTab.add("Angle converted", getArmExtension()).getEntry();
        // Targets
        armExtTargetEntry = armExtTab.add("Target", prevSetpointRaw).getEntry();
        armExtSetpointClampedEntry = armExtTab.add("Clamped setpoint", prevSetpointClamped).getEntry();
        // Misc.
        armExtMotorOutputEntry = armExtTab.add("Motor output", m_armExt.getAppliedOutput()).getEntry();
    }

    private void updateShuffleboardData() {
        // Booleans
        // Misc.
        armExtAtSetpointEntry.setBoolean(armExtensionAtSetpoint());
        armExtMotorInvertedEntry.setBoolean(m_armExt.getInverted());
        // Limits
        armExtAtUpperLimitEntry.setBoolean(armAtUpperLimit());
        armExtAtLowerLimitEntry.setBoolean(armAtLowerLimit());
        armExtEncoderLowerThanLimitEntry.setBoolean(debugEncoderAtLowerLimit());

        // Doubles
        // Angles
//        armExtRawEntry.setDouble(m_relativeEncoderArmEx.getPosition());
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

        prevSetpointRaw = targetExtRaw;
        prevSetpointClamped = targetExtClamped;

        if(armAtLowerLimit() && targetExtClamped <= 0) {
            m_armExt.set(0);
        } else {
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
        m_relativeEncoderArmEx.setPosition(-1.0);
    }

    public boolean armExtensionAtSetpoint() {
        return ((getArmExtension() >= prevSetpointClamped + Constants.ArmConstants.EXT_PID_TOLERANCE)
                || (getArmExtension() <= prevSetpointClamped - Constants.ArmConstants.EXT_PID_TOLERANCE));
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
    }

    public void updateInputs(ArmExtIOInputsAutoLogged inputs) {
        inputs.armExtension = getArmExtension();
    }
}

