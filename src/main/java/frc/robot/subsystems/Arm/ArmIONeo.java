package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
// import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;
import lib.utils.Utils;

public class ArmIONeo implements ArmIO {
    // private CANSparkMax m_ArmEx;
    private CANSparkMax m_armAngle;
    // private RelativeEncoder m_RelativeEncoderArmEx;
    private DutyCycleEncoder m_encoderArmAngle;
    private double kArmOffset = 260.9;

    // private SparkMaxPIDController m_sparkPID;
    private PIDController m_anglePID;

    public ArmIONeo() {
        // m_ArmEx = new CANSparkMax(ArmConstants.ArmExID, MotorType.kBrushed);
        m_armAngle = new CANSparkMax(ArmConstants.ArmAngleID, MotorType.kBrushless);
    
        m_encoderArmAngle = new DutyCycleEncoder(0);
        m_encoderArmAngle.reset();
        m_encoderArmAngle.setDistancePerRotation(360);
        m_encoderArmAngle.setPositionOffset(0);

        // m_sparkPID = m_armAngle.getPIDController();
        m_anglePID = new PIDController(ArmConstants.kPAngle, ArmConstants.kIAngle, ArmConstants.kDAngle);
        m_anglePID.enableContinuousInput(0, 360);
        // m_anglePID.setTolerance(100);
    }

    @Override
    public void updateInputs(ArmIOInputsAutoLogged inputs){
        inputs.ArmAngle = getArmAngle();

        inputs.ArmExtensionLength = getArmExtension();
    }

    @Override
    public void setAngleSpeed(double speed) {
        m_armAngle.set(speed);
    }

    @Override
    public void setArmSpeed(double speed) {
        // m_ArmEx.set(speed);
    }

    @Override
    public double getArmExtension() {
        // return m_RelativeEncoderArmEx.getPosition() * 360;
        return 0.0;
    }

    @Override
    public void setArmAngle(double angle){
        double angleSetpoint = MathUtil.clamp(angle, ArmConstants.kReverseLimit, ArmConstants.kForwardLimit);
        double pidOutput = MathUtil.clamp(m_anglePID.calculate(getArmAngle(), angleSetpoint), -3, 3);

        SmartDashboard.putNumber("Setpoint", angleSetpoint);
        SmartDashboard.putNumber("PID Output", pidOutput);

        if(!m_anglePID.atSetpoint()){
            m_armAngle.setVoltage(pidOutput);
        }
    }

    @Override
    public double getArmAngle() {
        // double angle = 0;
        SmartDashboard.putBoolean("IsConnected?", m_encoderArmAngle.isConnected());
        return Utils.normalize((m_encoderArmAngle.getAbsolutePosition() * 360) - kArmOffset);

        // return angle;
    }

}
