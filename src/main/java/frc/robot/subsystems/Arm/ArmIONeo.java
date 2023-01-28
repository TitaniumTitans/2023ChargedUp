package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
// import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;
import lib.utils.Swerve.CTREModuleState;

public class ArmIONeo implements ArmIO {
    // private CANSparkMax m_ArmEx;
    private CANSparkMax m_ArmAngle;
    // private RelativeEncoder m_RelativeEncoderArmEx;
    private DutyCycleEncoder m_EncoderArmAngle;
    private double kArmOffset = Units.degreesToRotations(260.9);


    public ArmIONeo() {
        // m_ArmEx = new CANSparkMax(ArmConstants.ArmExID, MotorType.kBrushed);
        m_ArmAngle = new CANSparkMax(ArmConstants.ArmAngleID, MotorType.kBrushless);
    
        m_EncoderArmAngle = new DutyCycleEncoder(0);
        m_EncoderArmAngle.reset();
        m_EncoderArmAngle.setDistancePerRotation(360);
        m_EncoderArmAngle.setPositionOffset(0);
        // m_EncoderArmAngle.setMinRate(10);
        // m_EncoderArmAngle.setSamplesToAverage(5);
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
        // System.out.println(m_EncoderArmAngle.getDistance());
        SmartDashboard.putNumber("Arm Angle", m_EncoderArmAngle.getDistance());
        SmartDashboard.putBoolean("Encoder Connected", m_EncoderArmAngle.isConnected());
        SmartDashboard.putNumber("Encoder Absoltue", (m_EncoderArmAngle.getAbsolutePosition() - kArmOffset) * 360);

        double angle = 0;
        CTREModuleState.placeInAppropriate0To360Scope((m_EncoderArmAngle.getAbsolutePosition() - kArmOffset) * 360, angle);

        return angle;
    }

}
