package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;
import lib.utils.Utils;

public class ArmIONeo implements ArmIO {
    private final CANSparkMax m_armEx;
    private final CANSparkMax m_armAngleMaster;
    private final CANSparkMax m_armAngleFollower;
    private final RelativeEncoder m_relativeEncoderArmEx;
    private final DutyCycleEncoder m_encoderArmAngle;
    private final DigitalInput m_armLimitSwitch;

    private final PIDController m_anglePID;
    private final PIDController m_extPID;

    public ArmIONeo() {
        m_armEx = new CANSparkMax(ArmConstants.ARM_EXTENSION_ID, MotorType.kBrushless);
        m_armAngleMaster = new CANSparkMax(ArmConstants.ARM_ANGLE_ID_MASTER, MotorType.kBrushless);
        m_armAngleFollower = new CANSparkMax(ArmConstants.ARM_ANGLE_ID_FOLLOWER, MotorType.kBrushless);

        m_armEx.setIdleMode(IdleMode.kBrake);
        m_armAngleMaster.setInverted(false);
        m_armAngleFollower.setInverted(false);

        m_armAngleMaster.setIdleMode(IdleMode.kBrake);
        m_armAngleFollower.setIdleMode(IdleMode.kBrake);

        m_armAngleFollower.follow(m_armAngleMaster);

        m_relativeEncoderArmEx = m_armEx.getEncoder();

        m_armAngleMaster.setIdleMode(IdleMode.kBrake);
        m_armAngleFollower.setIdleMode(IdleMode.kBrake);
    
        m_encoderArmAngle = new DutyCycleEncoder(ArmConstants.ENCODER_PORT);
        m_encoderArmAngle.reset();
        m_encoderArmAngle.setDistancePerRotation(360);
        m_encoderArmAngle.setPositionOffset(0);

        m_anglePID = new PIDController(ArmConstants.KP_ANGLE, ArmConstants.KI_ANGLE, 0.0);
         m_anglePID.setTolerance(3);

        m_armLimitSwitch = new DigitalInput(ArmConstants.LIMIT_SWITCH_PORT);
        m_extPID = new PIDController(ArmConstants.ARM_EXT_KP, ArmConstants.ARM_EXT_KI, ArmConstants.ARM_EXT_KD);
        m_extPID.setTolerance(0.5);
    }

    @Override
    public void updateInputs(ArmIOInputsAutoLogged inputs){
        inputs.ArmAngle = getArmAngle();

        inputs.ArmExtensionLength = getArmExtension();
    }

    @Override
    public void setAngleSpeed(double speed) {
        if ((getArmAngle() <= ArmConstants.K_REVERSE_LIMIT && speed <= 0)
            || (getArmAngle() >= ArmConstants.K_FORWARD_LIMIT && speed >= 0)) {
            m_armAngleMaster.set(speed);
        }
    }

    @Override
    public void setArmExtensionSpeed(double speed) {
        if(armAtLowerLimit() && speed <= 0){
            m_armEx.set(0);
        } else 
        {
            m_armEx.set(speed);
        }
    }

    @Override
    public void setArmExtension(double extension) {
        double setpoint = MathUtil.clamp(extension, ArmConstants.EXT_LOWER_LIMIT, ArmConstants.EXT_HIGHER_LIMIT);
        double pidOutput = MathUtil.clamp(m_extPID.calculate(getArmExtension(), setpoint), -0.5, 0.5);

        SmartDashboard.putNumber("Exstention Setpoint", setpoint);
        SmartDashboard.putNumber("PID Output", pidOutput);
        SmartDashboard.putBoolean("At Lower Limit", armAtLowerLimit());

        if(armAtLowerLimit() && pidOutput <= 0){
            m_armEx.set(0);
        } else {
            m_armEx.set(pidOutput);
        }
    }

    @Override
    public double getArmExtension() {
        return m_relativeEncoderArmEx.getPosition() * ArmConstants.EXTENSION_RATIO;
    }

    @Override
    public void setArmAngle(double angle) {
        double currentArmAngle = getArmAngle();
        double angleSetpoint = MathUtil.clamp(angle, ArmConstants.K_REVERSE_LIMIT, ArmConstants.K_FORWARD_LIMIT);
        double pidOutput = MathUtil.clamp(m_anglePID.calculate(currentArmAngle, angleSetpoint), -6, 6);

        SmartDashboard.putNumber("Arm Angle Setpoint", angleSetpoint);
        SmartDashboard.putNumber("Arm Angle PID output", pidOutput);
        SmartDashboard.putNumber("Current Arm Angle", currentArmAngle);

        m_armAngleMaster.setVoltage(-pidOutput);
    }

    @Override
    public double getArmAngle() {
        double armOffset = 260.9;
        return Utils.normalize((m_encoderArmAngle.getAbsolutePosition() * 360) - armOffset);

    }

    @Override
    public boolean encoderConnected() {
        return m_encoderArmAngle.isConnected();
    }

    @Override
    public boolean armAtLowerLimit() {
        return !m_armLimitSwitch.get();
    }

    @Override
    public void resetExstentionEncoder() {
        m_relativeEncoderArmEx.setPosition(0.0);
    }

    @Override
    public boolean armAngleAtSetpoint() {
        return m_anglePID.atSetpoint();
    }

    @Override
    public boolean armExstentionAtSetpoint() {
        return m_extPID.atSetpoint();
    }
}
