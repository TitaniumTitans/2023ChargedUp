package frc.robot.subsystems.wrist;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class WristSubsystem extends SubsystemBase {
    private CANSparkMax m_wristMotor;
    private CANSparkMax m_intakeMotor;
    private DigitalInput m_wristZeroLimit;
    private CANCoder m_wristEncoder;
    private PIDController m_wristPID;
    private TimeOfFlight m_tofSensor;
    private WristIOInputsAutoLogged m_input;

    @AutoLog
    public static class WristIOInputs {
        public double wristAngle = 0.0;
        public double intakeAmps = 0.0;
    }

    public WristSubsystem() {
        m_wristMotor = new CANSparkMax(WristConstants.WRIST_ID, MotorType.kBrushless);
        m_intakeMotor = new CANSparkMax(WristConstants.INTAKE_ID, MotorType.kBrushless);

        m_wristEncoder = new CANCoder(WristConstants.WRIST_ANGLE_PORT);
        m_wristEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);

        m_wristZeroLimit = new DigitalInput(WristConstants.LIMIT_SWITCH_PORT);

        m_tofSensor = new TimeOfFlight(WristConstants.TOF_PORT);
        m_tofSensor.setRangingMode(RangingMode.Short, 10);

        m_wristPID = new PIDController(WristConstants.WRIST_KP, WristConstants.WRIST_KI, WristConstants.WRIST_KD);
        m_wristPID.setTolerance(2);
    }

    @Override
    public void periodic() {
        updateInputs(m_input);
        Logger.getInstance().processInputs("Wrist", m_input);

        if (atLimit()) {
            zeroWristAngle();
        }
    }

    public void updateInputs(WristIOInputsAutoLogged inputs){
        inputs.intakeAmps = getIntakeAmps();
        inputs.wristAngle = getWristAngle();
    }
    
    //Setters
    public void setWristAngle(double angle) {

        double currentWristAngle = getWristAngle();
        double setpoint = MathUtil.clamp(angle, WristConstants.WRIST_LOWER_LIMIT, WristConstants.WRIST_UPPER_LIMIT);
        double output = MathUtil.clamp(m_wristPID.calculate(currentWristAngle, setpoint), -0.25, 0.25);


//        SmartDashboard.putNumber("Actual Wrist Angle", currentWristAngle);
//        SmartDashboard.putNumber("Target Angle", angle);
//        SmartDashboard.putNumber("Clamped Target Angle", setpoint);
//        SmartDashboard.putNumber("PID Output", output);

        m_wristMotor.set(output);
    }

    public void setWristPower(double speed) {

        if (atLimit() && speed <= 0){
            m_wristMotor.set(0.0);
        } else if (getWristAngle() >= WristConstants.WRIST_UPPER_LIMIT && speed >= 0) {
            m_wristMotor.set(0.0);
        } else {
            m_wristMotor.set(speed);
        }
    }

    public Command setWristPowerFactory(double speed) {
        return runOnce(() -> {setWristPower(speed);});
    }

    public void setIntakeSpeed(double speed) {
        m_intakeMotor.set(speed);
    }

    public Command setIntakeSpeedFactory(double speed) {
        return runOnce(() -> {setIntakeSpeed(speed);});
    }

    public void zeroWristAngle() {
        if (atLimit()) {
            m_wristEncoder.setPosition(0);
        }
    }

    //Getters
    public double getWristAngle() {
        return m_wristEncoder.getPosition() / WristConstants.WRIST_PIVOT_RATIO;
    }

    public double getIntakeAmps() {
        return m_intakeMotor.getOutputCurrent();
    }

    public boolean atLimit() {
        return m_wristZeroLimit.get();
    }

    public boolean pieceInside() {
        return m_tofSensor.getRange() < 1000;
    }

    public double getDetectionRange() {
        return m_tofSensor.getRange();
    }

    public boolean wristAtSetpoint() {
        return m_wristPID.atSetpoint();
    }
}
