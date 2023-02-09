package frc.robot.subsystems.Wrist;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.WristConstants;

public class WristIONeo implements WristIO{
    private CANSparkMax m_wristMotor;
    private CANSparkMax m_intakeMotor;
    private DigitalInput m_wristZeroLimit;
    private CANCoder m_wristEncoder;
    private PIDController m_wristPID;

    public WristIONeo() {
        m_wristMotor = new CANSparkMax(WristConstants.WRIST_ID, MotorType.kBrushless);
        m_intakeMotor = new CANSparkMax(WristConstants.INTAKE_ID, MotorType.kBrushless);

        m_wristEncoder = new CANCoder(WristConstants.WRIST_ANGLE_PORT);
        m_wristEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);

        m_wristZeroLimit = new DigitalInput(WristConstants.LIMIT_SWTICH_PORT);

        m_wristPID = new PIDController(WristConstants.WRIST_KP, WristConstants.WRIST_KI, WristConstants.WRIST_KD);
    }
    
    @Override
    public void updateInputs(WristIOInputsAutoLogged inputs){
        inputs.intakeAmps = getIntakeAmps();
        inputs.wristAngle = getWristAngle();
    }
    
    //Setters
    @Override
    public void setWristAngle(double angle) {
        double setpoint = MathUtil.clamp(angle, WristConstants.WRIST_LOWER_LIMIT, WristConstants.WRIST_UPPER_LIMIT);
    }

    @Override
    public void setWristPower(double speed) {
        m_wristMotor.set(speed);
            
    }

    @Override
    public void setIntakeSpeed(double speed) {
        m_intakeMotor.set(speed);
    }

    @Override
    public void zeroWristAngle() {
        if (atLimit()) {
            m_wristEncoder.setPosition(0);
        }
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
        return m_wristZeroLimit.get();
    }

}
