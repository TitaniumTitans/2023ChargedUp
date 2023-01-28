package frc.robot.subsystems.Arm;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants.ArmConstants;

public class ArmIONeo implements ArmIO {
    private CANSparkMax m_ArmEx;
    private CANSparkMax m_ArmAngle;
    private RelativeEncoder m_RelativeEncoderArmEx;
    private Encoder m_EncoderArmAngle;

    public ArmIONeo() {
        m_ArmEx = new CANSparkMax(ArmConstants.ArmExID, MotorType.kBrushed);
        m_ArmAngle = new CANSparkMax(ArmConstants.ArmAngleID, MotorType.kBrushless);
    
        m_EncoderArmAngle = new Encoder(0,1);
        m_EncoderArmAngle.setDistancePerPulse(ArmConstants.kAngleConversionFactor);
    }

    @Override
    public void updateInputs(ArmIOInputsAutoLogged inputs){
        inputs.ArmAngle = getArmAngle();

        inputs.ArmExtensionLength = getArmExtension();
    }

    @Override
    public void setAngleSpeed(double speed) {
        m_ArmAngle.set(speed);
    }

    @Override
    public void setArmSpeed(double speed) {
        m_ArmEx.set(speed);
    }

    @Override
    public double getArmExtension() {
        return m_RelativeEncoderArmEx.getPosition() * 360;
    }

    @Override
    public double getArmAngle() {
        return m_EncoderArmAngle.getDistance();
    }

}
