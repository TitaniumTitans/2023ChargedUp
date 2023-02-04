package frc.robot.subsystems.Wrist;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.IntakeConstants;

public class WristIONeo implements WristIO{
    private CANSparkMax m_wristMotor;
    private CANSparkMax m_intakeMotor;
    private DigitalInput m_zeroLimit;
    private CANCoder m_wristEncoder;

    public WristIONeo() {
        m_wristMotor = new CANSparkMax(IntakeConstants.WRIST_ID, MotorType.kBrushless);
        m_intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_ID, MotorType.kBrushless);

        m_wristEncoder = new CANCoder(IntakeConstants.WRIST_ANGLE_PORT);
        m_wristEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

        m_zeroLimit = new DigitalInput(IntakeConstants.LIMIT_SWTICH_PORT);
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

    @Override
    public void setWristPower(double speed) {
        m_wristMotor.set(speed);
    }

    @Override
    public void setIntakeSpeed(double speed) {
        m_intakeMotor.set(speed);
    }


    //Getters
    @Override
    public double getWristAngle() {
        return m_wristEncoder.getPosition();
    }

    @Override
    public double getIntakeAmps() {
        return m_intakeMotor.getOutputCurrent();
    }

    @Override
    public boolean atLimit() {
        return m_zeroLimit.get();
    }

}
