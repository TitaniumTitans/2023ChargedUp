package frc.robot.subsystems.wrist;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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

    //Shuffleboard data
    private ShuffleboardTab wristSubsystemTab;
    private GenericEntry wristAtSetpointEntry;
    private GenericEntry wristMotorInvertedEntry;
    private GenericEntry pieceInsideEntry;
    private GenericEntry wristAtUpperLimit;
    private GenericEntry wristAtLowerLimitEntry;
    private GenericEntry wristEncoderLowerThanLimitEntry;
    private GenericEntry wristAngleRawEntry;
    private GenericEntry wristAngleConvertedEntry;
    private GenericEntry wristTargetEntry;
    private GenericEntry wristSetpointClampedEntry;
    private GenericEntry wristPIDOutputEntry;
    private GenericEntry wristTOFSensorDistanceEntry;
    private GenericEntry wristMotorOutputEntry;
    private GenericEntry intakeMotorOutputEntry;

    @AutoLog
    public static class WristIOInputs {
        public double wristAngle = 0.0;
        public double intakeAmps = 0.0;
    }

    public WristSubsystem() {

        m_input = new WristIOInputsAutoLogged();

        SparkMaxFactory.SparkMaxConfig config = new SparkMaxFactory.SparkMaxConfig();
        config.setInverted(true);
        config.setIdleMode(CANSparkMax.IdleMode.kBrake);

        m_wristMotor  = SparkMaxFactory.Companion.createSparkMax(WristConstants.WRIST_ID, config);
        // Current limit based off testing 2/28/2023 17:55

        m_wristMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        config.setCurrentLimit(10);
        config.setInverted(false);
        m_intakeMotor  = SparkMaxFactory.Companion.createSparkMax(WristConstants.INTAKE_ID, config);

        m_wristEncoder = new CANCoder(WristConstants.WRIST_ANGLE_PORT);
        m_wristEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
        m_wristEncoder.configSensorDirection(true);

        m_wristZeroLimit = new DigitalInput(WristConstants.LIMIT_SWITCH_PORT);

        m_tofSensor = new TimeOfFlight(WristConstants.TOF_PORT);
        m_tofSensor.setRangingMode(RangingMode.Short, 10);

        m_wristPID = new PIDController(WristConstants.WRIST_KP, WristConstants.WRIST_KI, WristConstants.WRIST_KD);
        m_wristPID.setTolerance(2);

        wristSubsystemTab = Shuffleboard.getTab("WristSubsystem");

        addShuffleboardData();
    }

    private void addShuffleboardData() {
        // Booleans
        // Misc.
        wristAtSetpointEntry = wristSubsystemTab.add("At setpoint", wristAtSetpoint()).getEntry();
        wristMotorInvertedEntry = wristSubsystemTab.add("Motor inverted", m_wristMotor.getInverted()).getEntry();
        pieceInsideEntry = wristSubsystemTab.add("Piece inside", pieceInside()).getEntry();
        // Limits
        wristAtUpperLimit = wristSubsystemTab.add("At upper limit", wristAtUpperLimit()).getEntry();
        wristAtLowerLimitEntry = wristSubsystemTab.add("Limit switch triggered", atLowerLimit()).getEntry();
        wristEncoderLowerThanLimitEntry = wristSubsystemTab.add("Wrist encoder lower than limit", debugWristLowerThanLimit()).getEntry();

        // Doubles
        // Angles
        wristAngleRawEntry = wristSubsystemTab.add("Angle raw", m_wristEncoder.getPosition()).getEntry();
        wristAngleConvertedEntry = wristSubsystemTab.add("Angle converted", getWristAngle()).getEntry();
        // Targets
        wristTargetEntry = wristSubsystemTab.add("Target", prevSetpointRaw).getEntry();
        wristSetpointClampedEntry = wristSubsystemTab.add("Clamped setpoint", prevSetpointClamped).getEntry();
        wristPIDOutputEntry = wristSubsystemTab.add("PID setpoint output", prevSetpointPID).getEntry();
        // Misc.
        wristTOFSensorDistanceEntry = wristSubsystemTab.add("TOF detection range", getDetectionRange()).getEntry();
        wristMotorOutputEntry = wristSubsystemTab.add("Motor output", m_wristMotor.getAppliedOutput()).getEntry();
        intakeMotorOutputEntry = wristSubsystemTab.add("Intake motor output", m_intakeMotor.getAppliedOutput()).getEntry();
    }
    private void updateShuffleboardData() {
        // Booleans
        // Misc.
    wristAtSetpointEntry.setBoolean(wristAtSetpoint());
        pieceInsideEntry.setBoolean(pieceInside());
        // Limits
        wristAtUpperLimit.setBoolean(wristAtUpperLimit());
        wristAtLowerLimitEntry.setBoolean(atLowerLimit());
        wristEncoderLowerThanLimitEntry.setBoolean(debugWristLowerThanLimit());

        // Doubles
        // Angles
        wristAngleRawEntry.setDouble(m_wristEncoder.getPosition());
        wristAngleConvertedEntry.setDouble(getWristAngle());
        // Targets
        wristTargetEntry.setDouble(prevSetpointRaw);
        wristSetpointClampedEntry.setDouble(prevSetpointClamped);
        wristPIDOutputEntry.setDouble(prevSetpointPID);
        // Misc.
        wristTOFSensorDistanceEntry.setDouble(getDetectionRange());
        wristMotorOutputEntry.setDouble(m_wristMotor.getAppliedOutput());
        intakeMotorOutputEntry.setDouble(m_intakeMotor.getAppliedOutput());
    }

    @Override
    public void periodic() {
        //updateInputs(m_input);
        //Logger.getInstance().processInputs("Arm Wrist", m_input);

        updateShuffleboardData();

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

        double targetAngleClamped = MathUtil.clamp(targetAngleRaw, Constants.LimitConstants.WRIST_SCORE_LOWER.getValue(), Constants.LimitConstants.WRIST_SCORE_UPPER.getValue());
        double targetAnglePID = MathUtil.clamp(m_wristPID.calculate(currentWristAngle, targetAngleClamped), -0.15, 0.15);

        // Dashboard variables
        prevSetpointRaw = targetAngleRaw;
        prevSetpointClamped = targetAngleClamped;
        prevSetpointPID = targetAnglePID;

        m_wristMotor.set(targetAnglePID);
    }

    public void setWristPower(double speed) {

        if (atLowerLimit() && speed <= 0){
            m_wristMotor.set(0.0);
        } else if (getWristAngle() >= Constants.LimitConstants.WRIST_SCORE_UPPER.getValue() && speed >= 0) {
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
        return (getWristAngle() >= Constants.LimitConstants.WRIST_SCORE_UPPER.getValue());
    }

    public boolean debugWristLowerThanLimit() {
        return (getWristAngle() <= Constants.LimitConstants.WRIST_SCORE_LOWER.getValue());
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
