package frc.robot.subsystems.arm;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmExtSubsystem extends SubsystemBase {
    private final CANSparkMax m_ArmEx;
    private final RelativeEncoder m_relativeEncoderArmEx;
    private final PIDController m_extPID;
    private final DigitalInput m_armLimitSwitch;
    // Logging variables
    private double prevSetpointRaw;
    private double prevSetpointClamped;
    private double prevSetpointPID;
    private ShuffleboardTab armExtTab;

    public ArmExtSubsystem() {
        m_ArmEx = new CANSparkMax(Constants.ArmConstants.ARM_EXTENSION_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        m_ArmEx.setIdleMode(CANSparkMax.IdleMode.kBrake);

        m_relativeEncoderArmEx = m_ArmEx.getEncoder();

        m_extPID = new PIDController(Constants.ArmConstants.ARM_EXT_KP, Constants.ArmConstants.ARM_EXT_KI, Constants.ArmConstants.ARM_EXT_KD);
        m_extPID.setTolerance(0.5);

        m_armLimitSwitch = new DigitalInput(Constants.ArmConstants.LIMIT_SWITCH_PORT);

        armExtTab = Shuffleboard.getTab("ArmExtSubsystem");
    }

    public void setArmSpeed(double speed) {
        if(armAtLowerLimit() && speed <= 0){
            m_ArmEx.set(0);
        } else
        {
            m_ArmEx.set(speed);
        }
    }

    public Command setArmSpeedFactory(double speed) {
        return runOnce(() -> {setArmSpeed(speed);});
    }

    public void setArmExtension(double targetExtRaw) {
        double targetExtClamped = MathUtil.clamp(targetExtRaw, Constants.ArmConstants.EXT_LOWER_LIMIT, Constants.ArmConstants.EXT_HIGHER_LIMIT);
        double targetExtPID = MathUtil.clamp(m_extPID.calculate(getArmExtension(), targetExtClamped), -0.5, 0.5);

        prevSetpointRaw = targetExtRaw;
        prevSetpointClamped = targetExtClamped;
        prevSetpointPID = targetExtPID;

        if(armAtLowerLimit() && targetExtPID <= 0){
            m_ArmEx.set(0);
        } else {
            m_ArmEx.set(targetExtPID);
        }
    }

    public double getArmExtension() {
        return m_relativeEncoderArmEx.getPosition() * Constants.ArmConstants.EXTENSION_RATIO;
    }

    public boolean armAtLowerLimit() {
        return !m_armLimitSwitch.get();
    }

    public boolean armAtUpperLimit() {
        return (m_relativeEncoderArmEx.getPosition() >= Constants.ArmConstants.EXT_HIGHER_LIMIT);
    }

    public boolean debugEncoderAtLowerLimit() {
        return (m_relativeEncoderArmEx.getPosition() <= Constants.ArmConstants.EXT_LOWER_LIMIT);
    }

    public void resetExtensionEncoder() {
        m_relativeEncoderArmEx.setPosition(0.0);
    }

    public boolean armExtensionAtSetpoint() {
        return m_extPID.atSetpoint();
    }

    @Override
    public void periodic() {
        // Booleans
        // Misc.
        armExtTab.add("At setpoint", armExtensionAtSetpoint());
        armExtTab.add("Motor inverted", m_ArmEx.getInverted());
        // Limits
        armExtTab.add("Limit switch triggered", armAtLowerLimit());
        armExtTab.add("At upper limit", armAtUpperLimit());
        armExtTab.add("At encoder lower than limit", debugEncoderAtLowerLimit());

        // Doubles
        // Angles
        armExtTab.add("Encoder raw",m_relativeEncoderArmEx);
        armExtTab.add("Extension converted", getArmExtension());
        // Targets
        armExtTab.add("Target", prevSetpointRaw);
        armExtTab.add("Clamped setpoint", prevSetpointClamped);
        armExtTab.add("PID setpoint output", prevSetpointPID);
    }
}

