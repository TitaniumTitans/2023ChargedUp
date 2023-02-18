package frc.robot.subsystems.wrist;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;
import lib.factories.SparkMaxFactory;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class WristSubsystem extends SubsystemBase {
    private final CANSparkMax m_wristMotor;
    private final CANSparkMax m_intakeMotor;
    private final DigitalInput m_wristZeroLimit;
    private final CANCoder m_wristEncoder;
    private final PIDController m_wristPID;
    private final TimeOfFlight m_tofSensor;
    private final WristIOInputsAutoLogged m_input;
    // Logging variables
    private double prevSetpointRaw;
    private double prevSetpointClamped;
    private double prevSetpointPID;
    private ShuffleboardTab wristAngleTab;

    @AutoLog
    public static class WristIOInputs {
        public double wristAngle = 0.0;
        public double intakeAmps = 0.0;
    }

    public WristSubsystem() {

        m_input = new WristIOInputsAutoLogged();

        SparkMaxFactory.SparkMaxConfig config = new SparkMaxFactory.SparkMaxConfig();

        m_wristMotor  = SparkMaxFactory.Companion.createSparkMax(WristConstants.WRIST_ID, config);
        m_intakeMotor  = SparkMaxFactory.Companion.createSparkMax(WristConstants.INTAKE_ID, config);

        m_wristEncoder = new CANCoder(WristConstants.WRIST_ANGLE_PORT);
        m_wristEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);

        m_wristZeroLimit = new DigitalInput(WristConstants.LIMIT_SWITCH_PORT);

        m_tofSensor = new TimeOfFlight(WristConstants.TOF_PORT);
        m_tofSensor.setRangingMode(RangingMode.Short, 10);

        m_wristPID = new PIDController(WristConstants.WRIST_KP, WristConstants.WRIST_KI, WristConstants.WRIST_KD);
        m_wristPID.setTolerance(2);

        wristAngleTab = Shuffleboard.getTab("WristSubsystem");
    }

    @Override
    public void periodic() {
        updateInputs(m_input);
        Logger.getInstance().processInputs("Wrist", m_input);

        // Booleans
        // Misc.
        wristAngleTab.add("At setpoint", wristAtSetpoint());
        wristAngleTab.add("Motor inverted", m_wristMotor.getInverted());
        wristAngleTab.add("Piece inside", pieceInside());
        // Limits
        wristAngleTab.add("At upper limit", wristAtUpperLimit());
        wristAngleTab.add("Limit switch triggered", atLowerLimit());
        wristAngleTab.add("Wrist encoder lower than limit", debugWristLowerThanLimit());

        // Doubles
        // Angles
        wristAngleTab.add("Angle raw", m_wristEncoder.getPosition());
        wristAngleTab.add("Angle converted", getWristAngle());
        // Targets
        wristAngleTab.add("Target", prevSetpointRaw);
        wristAngleTab.add("Clamped setpoint", prevSetpointClamped);
        wristAngleTab.add("PID setpoint output", prevSetpointPID);
        // Misc.
        wristAngleTab.add("TOF detection range", getDetectionRange());

        if (atLowerLimit()) {
            zeroWristAngle();
        }
    }

    public void updateInputs(WristIOInputsAutoLogged inputs){
        inputs.intakeAmps = getIntakeAmps();
        inputs.wristAngle = getWristAngle();
    }
    
    //Setters
    public void setWristAngle(double targetAngleRaw) {
        double currentWristAngle = getWristAngle();

        double targetAngleClamped = MathUtil.clamp(targetAngleRaw, WristConstants.WRIST_LOWER_LIMIT, WristConstants.WRIST_UPPER_LIMIT);
        double targetAnglePID = MathUtil.clamp(m_wristPID.calculate(currentWristAngle, targetAngleClamped), -0.25, 0.25);

        // Dashboard variables
        prevSetpointRaw = targetAngleRaw;
        prevSetpointClamped = targetAngleClamped;
        prevSetpointPID = targetAnglePID;

        m_wristMotor.set(targetAnglePID);
    }

    public void setWristPower(double speed) {

        if (atLowerLimit() && speed <= 0){
            m_wristMotor.set(0.0);
        } else if (getWristAngle() >= WristConstants.WRIST_UPPER_LIMIT && speed >= 0) {
            m_wristMotor.set(0.0);
        } else {
            m_wristMotor.set(speed);
        }
    }

    public Command setWristPowerFactory(double speed) {
        return runOnce(() -> setWristPower(speed));
    }

    public void setIntakeSpeed(double speed) {
        m_intakeMotor.set(speed);
    }

    public Command setIntakeSpeedFactory(double speed) {
        return runOnce(() -> setIntakeSpeed(speed));
    }

    public void zeroWristAngle() {
        if (atLowerLimit()) {
            m_wristEncoder.setPosition(0);
        }
    }

    //Getters
    public double getWristAngle() {
        return m_wristEncoder.getPosition() / WristConstants.WRIST_PIVOT_RATIO;
    }

    public boolean wristAtUpperLimit() {
        return (getWristAngle() >= WristConstants.WRIST_UPPER_LIMIT);
    }

    public boolean debugWristLowerThanLimit() {
        return (getWristAngle() <= WristConstants.WRIST_LOWER_LIMIT);
    }


    public double getIntakeAmps() {
        return m_intakeMotor.getOutputCurrent();
    }

    public boolean atLowerLimit() {
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
