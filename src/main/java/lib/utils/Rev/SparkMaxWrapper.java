package lib.utils.Rev;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import lib.utils.drivers.RevUtil;

public class SparkMaxWrapper {

    private final CANSparkMax m_motor;
    private final SparkMaxPIDController m_controller;
    private final RelativeEncoder m_relativeEncoder;

    public enum ControlMode {
        PERCENT_OUTPUT,
        VELOCITY,
        POSITION
    }

    //TODO maybe implement brushless motor/alternative encoders
    public SparkMaxWrapper(int id) {
        m_motor = new CANSparkMax(id, MotorType.kBrushless);
        m_relativeEncoder = m_motor.getEncoder();
        m_controller = m_motor.getPIDController();

        SparkMaxConfigs.configCanStatusFrames(m_motor);
    }

    // Motor methods
    // TODO finish implementing the rest of the control modes
    public void set(ControlMode mode, double speed) {
        switch (mode) {
            case PERCENT_OUTPUT:
                m_motor.set(speed);
                break;
            case VELOCITY:
                m_controller.setReference(speed, ControlType.kVelocity);
                break;
            case POSITION:
                m_controller.setReference(speed, ControlType.kPosition);
                break;
            default:
                m_motor.set(0.0);
        }
    }

    public void follow(SparkMaxWrapper master) {
        m_motor.follow(master.getMotor());
    }

    public void follow(CANSparkMax master) {
        m_motor.follow(master);
    }

    public void setInvert(boolean invert) {
        m_motor.setInverted(invert);
    }

    public boolean getInvert() {
        return m_motor.getInverted();
    }

    public void setIdleMode(CANSparkMax.IdleMode idleMode) {
        m_motor.setIdleMode(idleMode);
    }

    // PID methods
    public void setPIDGains(double kP, double kI, double kD) {
        RevUtil.autoRetry(() -> m_controller.setP(kP));
        RevUtil.autoRetry(() -> m_controller.setI(kI));
        RevUtil.autoRetry(() -> m_controller.setD(kD));
    }

    // Encoder methods
    public void resetEncoder() {
        RevUtil.autoRetry(() -> m_relativeEncoder.setPosition(0));
    }

    public void setPostionConversion(double conversionFactor) {
        RevUtil.autoRetry(() ->m_relativeEncoder.setPositionConversionFactor(conversionFactor));
    }

    public void setVelocityConversion(double conversionFactor) {
        RevUtil.autoRetry(() -> m_relativeEncoder.setVelocityConversionFactor(conversionFactor));
    }

    public double getPosition() {
        return m_relativeEncoder.getPosition();
    }

    public double getVelocity() {
        return m_relativeEncoder.getVelocity();
    }



    //TODO remove these next few methods, these are just for
    //TODO any extra methods that I haven't gotten to yet
    public CANSparkMax getMotor() {
        return m_motor;
    }

    public RelativeEncoder getEncoder() {
        return m_relativeEncoder;
    }

    public SparkMaxPIDController getController() {
        return m_controller;
    }
}
