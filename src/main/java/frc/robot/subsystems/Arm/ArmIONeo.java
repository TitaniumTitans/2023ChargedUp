package frc.robot.subsystems.Arm;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ArmConstants;

public class ArmIONeo implements ArmIO {
    // private CANSparkMax m_ArmEx;
    private CANSparkMax m_ArmAngle;
    // private RelativeEncoder m_RelativeEncoderArmEx;
    private RelativeEncoder m_RelativeEncoderArmAngle;



    public ArmIONeo() {
        // m_ArmEx = new CANSparkMax(ArmConstants.ArmExID, MotorType.kBrushed);
        m_ArmAngle = new CANSparkMax(ArmConstants.ArmAngleID, MotorType.kBrushless);
    
        m_RelativeEncoderArmAngle = m_ArmAngle.getEncoder();
        m_RelativeEncoderArmAngle.setPositionConversionFactor(ArmConstants.kAngleConversionFactor);

        // m_ArmAngle.enableSoftLimit(SoftLimitDirection.kForward, true);
        // m_ArmAngle.enableSoftLimit(SoftLimitDirection.kReverse, true);
        // m_ArmAngle.setSoftLimit(SoftLimitDirection.kForward,10);
        // m_ArmAngle.setSoftLimit(SoftLimitDirection.kReverse, 180);
        // m_ArmAngle.setSoftLimit(SoftLimitDirection.kForward, (float) Units.degreesToRotations(10));
        // m_ArmAngle.setSoftLimit(SoftLimitDirection.kReverse, (float) Units.degreesToRotations(273.5));
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
        // m_ArmEx.set(speed);
    }

    @Override
    public double getArmExtension() {
        // return m_RelativeEncoderArmEx.getPosition() * 360;
        return 0.0;
    }

    @Override
    public double getArmAngle() {
        return Units.rotationsToDegrees(m_RelativeEncoderArmAngle.getPosition()) * 360;
    }

}
