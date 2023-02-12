package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
// import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
// import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;
import lib.utils.Utils;

public class ArmIONeo implements ArmIO {
    private CANSparkMax m_ArmEx;
    private CANSparkMax m_armAngleMaster;
    private CANSparkMax m_armAngleFollower;
    private RelativeEncoder m_relativeEncoderArmEx;
    public DutyCycleEncoder m_encoderArmAngle;
    private double kArmOffset = 260.9;
    private DigitalInput m_armLimitSwitch;

    // private SparkMaxPIDController m_sparkPID;
    private PIDController m_anglePID;
    private PIDController m_extPID;

    public ArmIONeo() {
        m_ArmEx = new CANSparkMax(ArmConstants.ARM_EXTENSION_ID, MotorType.kBrushless);
        m_armAngleMaster = new CANSparkMax(ArmConstants.ARM_ANGLE_ID_MASTER, MotorType.kBrushless);
        m_armAngleFollower = new CANSparkMax(ArmConstants.ARM_ANGLE_ID_FOLLOWER, MotorType.kBrushless);

        m_ArmEx.setIdleMode(IdleMode.kBrake);
        m_armAngleMaster.setInverted(false);
        m_armAngleFollower.setInverted(false);

        m_armAngleMaster.setIdleMode(IdleMode.kBrake);
        m_armAngleFollower.setIdleMode(IdleMode.kBrake);

        m_armAngleFollower.follow(m_armAngleMaster);

        m_relativeEncoderArmEx = m_ArmEx.getEncoder();

        m_armAngleMaster.setIdleMode(IdleMode.kBrake);
        m_armAngleFollower.setIdleMode(IdleMode.kBrake);
    
        m_encoderArmAngle = new DutyCycleEncoder(ArmConstants.ENCODER_PORT);
        m_encoderArmAngle.reset();
        m_encoderArmAngle.setDistancePerRotation(360);
        m_encoderArmAngle.setPositionOffset(0);

        // m_sparkPID = m_armAngle.getPIDController();
        m_anglePID = new PIDController(ArmConstants.KP_ANGLE, ArmConstants.KI_ANGLE, ArmConstants.KD_ANGLE);
        m_anglePID.enableContinuousInput(0, 360);
        // m_anglePID.setTolerance(100);

        m_armLimitSwitch = new DigitalInput(ArmConstants.LIMIT_SWITCH_PORT);
        m_extPID = new PIDController(ArmConstants.ARM_EXT_KP, ArmConstants.ARM_EXT_KI, ArmConstants.ARM_EXT_KD);
    }

    @Override
    public void updateInputs(ArmIOInputsAutoLogged inputs){
        inputs.ArmAngle = getArmAngle();

        inputs.ArmExtensionLength = getArmExtension();
    }

    @Override
    public void setAngleSpeed(double speed) {
        m_armAngleMaster.set(speed);
    }

    @Override
    public void setArmSpeed(double speed) {
        if(armAtLowerLimit() && speed <= 0){
            m_ArmEx.set(0);
        } else 
        {
            m_ArmEx.set(speed);
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
            m_ArmEx.set(0);
        } else
        {
            m_ArmEx.set(pidOutput);
        }
    }

    @Override
    public double getArmExtension() {
        return m_relativeEncoderArmEx.getPosition() * ArmConstants.EXTENSION_RATIO;
    }

    @Override
    public void setArmAngle(double angle){
        double angleSetpoint = MathUtil.clamp(angle, ArmConstants.K_REVERSE_LIMIT, ArmConstants.K_FORWARD_LIMIT);
        double pidOutput = MathUtil.clamp(m_anglePID.calculate(getArmAngle(), angleSetpoint), -3, 3);

        SmartDashboard.putNumber("Setpoint", angleSetpoint);
        SmartDashboard.putNumber("PID Output", pidOutput);

        m_armAngleMaster.setVoltage(pidOutput);
    }

    @Override
    public double getArmAngle() {
        // double angle = 0;
        SmartDashboard.putBoolean("IsConnected?", m_encoderArmAngle.isConnected());
        return Utils.normalize((m_encoderArmAngle.getAbsolutePosition() * 360) - kArmOffset);

        // return angle;
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
}
