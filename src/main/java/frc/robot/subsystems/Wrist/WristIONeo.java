package frc.robot.subsystems.Wrist;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.WristConstants;

public class WristIONeo implements WristIO{
    private CANSparkMax m_wristMotor;
    private CANSparkMax m_intakeMotor;
    private DigitalInput m_wristZeroLimit;
    private CANCoder m_wristEncoder;
    private TimeOfFlight m_tofSensor;

    public WristIONeo() {
        m_wristMotor = new CANSparkMax(WristConstants.WRIST_ID, MotorType.kBrushless);
        m_intakeMotor = new CANSparkMax(WristConstants.INTAKE_ID, MotorType.kBrushless);

        m_wristEncoder = new CANCoder(WristConstants.WRIST_ANGLE_PORT);
        m_wristEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

        m_wristZeroLimit = new DigitalInput(WristConstants.LIMIT_SWTICH_PORT);

        m_tofSensor = new TimeOfFlight(WristConstants.TOF_PORT);
        m_tofSensor.setRangingMode(RangingMode.Short, 10);
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
        // if(wristAtLowerLimit() && speed <= 0){
        //     m_wristMotor.set(0);
        // } else 
        // {
        //     m_wristMotor.set(speed);
        // }

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

    public boolean wristAtLowerLimit() {
        return m_wristZeroLimit.get();
    }

    @Override
    public boolean pieceInside() {
        return m_tofSensor.getRange() < 1000;
    }

    @Override
    public double getDetectionRange() {
        return m_tofSensor.getRange();
    }
}
