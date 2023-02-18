package frc.robot.subsystems.arm;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmExtSubsystem extends SubsystemBase {
    private final CANSparkMax m_ArmEx;
    private final RelativeEncoder m_relativeEncoderArmEx;
    private final PIDController m_extPID;
    private final DigitalInput m_armLimitSwitch;

    public ArmExtSubsystem() {
        m_ArmEx = new CANSparkMax(Constants.ArmConstants.ARM_EXTENSION_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        m_ArmEx.setIdleMode(CANSparkMax.IdleMode.kBrake);

        m_relativeEncoderArmEx = m_ArmEx.getEncoder();

        m_extPID = new PIDController(Constants.ArmConstants.ARM_EXT_KP, Constants.ArmConstants.ARM_EXT_KI, Constants.ArmConstants.ARM_EXT_KD);
        m_extPID.setTolerance(0.5);

        m_armLimitSwitch = new DigitalInput(Constants.ArmConstants.LIMIT_SWITCH_PORT);
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

    public void setArmExtension(double extension) {
        double setpoint = MathUtil.clamp(extension, Constants.ArmConstants.EXT_LOWER_LIMIT, Constants.ArmConstants.EXT_HIGHER_LIMIT);
        double pidOutput = MathUtil.clamp(m_extPID.calculate(getArmExtension(), setpoint), -0.5, 0.5);

        SmartDashboard.putNumber("Extension Setpoint", setpoint);
        SmartDashboard.putNumber("PID Output", pidOutput);
        SmartDashboard.putBoolean("At Lower Limit", armAtLowerLimit());

        if(armAtLowerLimit() && pidOutput <= 0){
            m_ArmEx.set(0);
        } else {
            m_ArmEx.set(pidOutput);
        }
    }

    public double getArmExtension() {
        return m_relativeEncoderArmEx.getPosition() * Constants.ArmConstants.EXTENSION_RATIO;
    }

    public boolean armAtLowerLimit() {
        return !m_armLimitSwitch.get();
    }

    public void resetExstentionEncoder() {
        m_relativeEncoderArmEx.setPosition(0.0);
    }

    public boolean armExstensionAtSetpoint() {
        return m_extPID.atSetpoint();
    }
}

