package frc.robot.subsystems.Wrist;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.IntakeConstants;

public class WristIONeo implements WristIO{
    private CANSparkMax m_wristAngle;
    private CANSparkMax m_intakePower;
    private DigitalInput m_zeroLimit;
    private DutyCycleEncoder m_wristAngleEncoder;

    public WristIONeo() {
        m_wristAngle = new CANSparkMax(IntakeConstants.WRIST_ID, MotorType.kBrushless);
        m_intakePower = new CANSparkMax(IntakeConstants.INTAKE_ID, MotorType.kBrushless);

        m_wristAngleEncoder = new DutyCycleEncoder(IntakeConstants.WRIST_ANGLE_PORT);
        m_wristAngleEncoder.setDistancePerRotation(360);
    }
    
    @Override
    public void updateInputs(WristIOInputsAutoLogged inputs){
        inputs.intakeAmps = getIntakeAmps();
        inputs.wristAngle = getWristAngle();
    }
    
    //Setters
    @Override
    public void setWristAngle(double angle) {
    }


    //Getters
    @Override
    public double getWristAngle() {
        return m_wristAngleEncoder.getDistance();
    }

    @Override
    public double getIntakeAmps() {
        return m_intakePower.getOutputCurrent();
    }

}
