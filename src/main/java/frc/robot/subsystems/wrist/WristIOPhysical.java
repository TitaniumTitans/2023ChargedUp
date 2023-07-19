package frc.robot.subsystems.wrist;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Constants.*;
import lib.factories.SparkMaxFactory;

public class WristIOPhysical implements WristIO{
    protected CANSparkMax m_wrist;
    private final CANCoder m_wristEncoder;
    private CANSparkMax m_intake;
    private PIDController m_pid;

    private DigitalInput m_lowerLimit;

    private boolean wristHomed = false;
    public WristIOPhysical() {
        // config for wrist
        SparkMaxFactory.SparkMaxConfig config = new SparkMaxFactory.SparkMaxConfig();
        config.setInverted(true);
        config.setIdleMode(CANSparkMax.IdleMode.kBrake);
        config.setCurrentLimit(20);

        m_wrist = SparkMaxFactory.Companion.createSparkMax(WristConstants.WRIST_ID, config);
        m_wrist.setOpenLoopRampRate(0.1);

        m_pid = new PIDController(WristConstants.WRIST_KP, WristConstants.WRIST_KI, WristConstants.WRIST_KD);

        // Wrist encoder
        m_wristEncoder = new CANCoder(WristConstants.WRIST_ANGLE_PORT);
        m_wristEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
        m_wristEncoder.configSensorDirection(true);


        // config for intake
        config.setCurrentLimit(40);
        config.setInverted(false);
        m_intake = SparkMaxFactory.Companion.createSparkMax(WristConstants.INTAKE_ID, config);

        // Sensors for limiting and piece detection
        m_lowerLimit = new DigitalInput(WristConstants.LIMIT_SWITCH_PORT);
    }

    @Override
    public void update(WristIOInputsAutoLogged inputs) {
        inputs.intakeAmpOutput = m_intake.getOutputCurrent();
        inputs.intakeStalling = m_intake.getFault(CANSparkMax.FaultID.kStall);
        inputs.intakeTemperature = m_intake.getMotorTemperature();

        inputs.wristAmpOutput = m_wrist.getOutputCurrent();
        inputs.wristAtSetpoint = m_pid.atSetpoint();
        inputs.wristAtLowerLimit = m_lowerLimit.get();
        inputs.wristAtUpperLimit = m_wrist.getEncoder().getPosition() > LimitConstants.WRIST_SCORE_UPPER.getValue();
        inputs.wristOutputRaw = m_wrist.getAppliedOutput();
        inputs.wristTemperature = m_wrist.getMotorTemperature();
        inputs.wristStalling = m_wrist.getFault(CANSparkMax.FaultID.kStall);
        inputs.wristAngle = m_wrist.getEncoder().getPosition();
        inputs.wristHomed = wristHomed;
    }

    @Override
    public void setIntakePower(double speed) {
        m_intake.set(speed == 0.0 ? 0.2 : speed);
    }

    @Override
    public void setWristAngle(double angle, double setpoint) {
        double clampedSetpoint = MathUtil.clamp(setpoint, Constants.LimitConstants.WRIST_SCORE_LOWER.getValue(), Constants.LimitConstants.WRIST_SCORE_UPPER.getValue());
        double targetAnglePID = MathUtil.clamp(m_pid.calculate(angle, clampedSetpoint), -0.7, 0.7);

        m_wrist.set(targetAnglePID);
    }

    @Override
    public void setWristPower(double speed) {

        if (m_lowerLimit.get() && speed <= 0){
            m_wrist.set(0.0);
        } else if (m_wristEncoder.getPosition() >= Constants.LimitConstants.WRIST_SCORE_UPPER.getValue() && speed >= 0) {
            m_wrist.set(0.0);
        } else {
            m_wrist.set(speed);
        }
    }

    @Override
    public void setBrakeMode(CANSparkMax.IdleMode brakeMode) {
        m_wrist.setIdleMode(brakeMode);
    }

    @Override
    public void toggleBrakeMode() {
        if (m_wrist.getIdleMode() == CANSparkMax.IdleMode.kBrake) {
            m_wrist.setIdleMode(CANSparkMax.IdleMode.kCoast);
        } else {
            m_wrist.setIdleMode(CANSparkMax.IdleMode.kBrake);
        }
    }

    @Override
    public void resetHomed() {
        wristHomed = false;
    }

    @Override
    public void goToHome() {
        if(!wristHomed) {
            if(m_lowerLimit.get()) {
                setWristPower(0);
                wristHomed = true;
            } else {
                setWristPower(-0.2);
            }
        } // endif
    }

    @Override
    public void zeroWrist() {
        m_wristEncoder.setPosition(0.0);
    }
}
