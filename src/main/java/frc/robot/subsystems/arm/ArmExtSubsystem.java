package frc.robot.subsystems.arm;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
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

public class ArmExtSubsystem extends SubsystemBase {
    private final CANSparkMax m_armExt;
    private final RelativeEncoder m_relativeEncoderArmEx;
    private final PIDController m_extPID;
    private final DigitalInput m_armLimitSwitch;
    // Logging variables
    private double prevSetpointRaw;
    private double prevSetpointClamped;
    private double prevSetpointPID;

    //Shuffleboard data
    private ShuffleboardTab armExtTab;
    private GenericEntry armExtAtSetpointEntry;
    private GenericEntry armExtMotorInvertedEntry;
    private GenericEntry armExtAtUpperLimitEntry;
    private GenericEntry armExtAtLowerLimitEntry;
    private GenericEntry armExtEncoderLowerThanLimitEntry;
    private GenericEntry armExtRawEntry;
    private GenericEntry armExtConvertedEntry;
    private GenericEntry armExtTargetEntry;
    private GenericEntry armExtSetpointClampedEntry;
    private GenericEntry armExtPIDOutputEntry;

    public ArmExtSubsystem() {
        SparkMaxFactory.SparkMaxConfig config = new SparkMaxFactory.SparkMaxConfig();

        m_armExt = SparkMaxFactory.Companion.createSparkMax(Constants.ArmConstants.ARM_EXTENSION_ID, config);

        m_relativeEncoderArmEx = m_armExt.getEncoder();
        m_relativeEncoderArmEx.setPositionConversionFactor(Constants.ArmConstants.EXTENSION_RATIO);

        m_extPID = new PIDController(Constants.ArmConstants.ARM_EXT_KP, Constants.ArmConstants.ARM_EXT_KI, Constants.ArmConstants.ARM_EXT_KD);
        m_extPID.setTolerance(0.5);

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
        armExtRawEntry = armExtTab.add("Angle raw", m_relativeEncoderArmEx.getPosition()).getEntry();
        armExtConvertedEntry = armExtTab.add("Angle converted", getArmExtension()).getEntry();
        // Targets
        armExtTargetEntry = armExtTab.add("Target", prevSetpointRaw).getEntry();
        armExtSetpointClampedEntry = armExtTab.add("Clamped setpoint", prevSetpointClamped).getEntry();
        armExtPIDOutputEntry = armExtTab.add("PID setpoint output", prevSetpointPID).getEntry();
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
        armExtRawEntry.setDouble(m_relativeEncoderArmEx.getPosition());
        armExtConvertedEntry.setDouble(getArmExtension());
        // Targets
        armExtTargetEntry.setDouble(prevSetpointRaw);
        armExtSetpointClampedEntry.setDouble(prevSetpointClamped);
        armExtPIDOutputEntry.setDouble(prevSetpointPID);
    }

    public void setArmSpeed(double speed) {
        if(armAtLowerLimit() && speed <= 0 || armAtUpperLimit() && speed >= 0){
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
        double targetExtPID = MathUtil.clamp(m_extPID.calculate(getArmExtension(), targetExtClamped), -0.5, 0.5);

        prevSetpointRaw = targetExtRaw;
        prevSetpointClamped = targetExtClamped;
        prevSetpointPID = targetExtPID;

        if(armAtLowerLimit() && targetExtPID <= 0){
            m_armExt.set(0);
        } else {
            m_armExt.set(targetExtPID);
        }
    }

    public double getArmExtension() {
        return m_relativeEncoderArmEx.getPosition() * Constants.ArmConstants.EXTENSION_RATIO;
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

    public boolean armExtensionAtSetpoint() {
        return m_extPID.atSetpoint();
    }

    @Override
    public void periodic() {
        updateShuffleboardData();
    }
}

