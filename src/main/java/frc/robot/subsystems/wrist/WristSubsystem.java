package frc.robot.subsystems.wrist;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private final WristIOInputsAutoLogged m_input;

    // Logging variables
    private double prevSetpointRaw;
    private double prevSetpointClamped;
    private double prevSetpointPID;
    public static boolean m_hasWristHomed = false;
    //Shuffleboard data
    private final ShuffleboardTab wristSubsystemTab;
    private GenericEntry wristAtSetpointEntry;
    private GenericEntry pieceInsideEntry;
    private GenericEntry wristAtUpperLimit;
    private GenericEntry wristAtLowerLimitEntry;
    private GenericEntry wristEncoderLowerThanLimitEntry;
    private GenericEntry wristAngleRawEntry;
    private GenericEntry wristAngleConvertedEntry;
    private GenericEntry wristTargetEntry;
    private GenericEntry wristSetpointClampedEntry;
    private GenericEntry wristPIDOutputEntry;
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
        config.setCurrentLimit(20);

        m_wristMotor  = SparkMaxFactory.Companion.createSparkMax(WristConstants.WRIST_ID, config);
        // Current limit based off testing 2/28/2023 17:55
        m_wristMotor.setOpenLoopRampRate(0.1);

        config.setCurrentLimit(40);
        config.setInverted(false);
        m_intakeMotor  = SparkMaxFactory.Companion.createSparkMax(WristConstants.INTAKE_ID, config);

        m_wristEncoder = new CANCoder(WristConstants.WRIST_ANGLE_PORT);
        m_wristEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
        m_wristEncoder.configSensorDirection(true);

        m_wristZeroLimit = new DigitalInput(WristConstants.LIMIT_SWITCH_PORT);

        m_wristPID = new PIDController(WristConstants.WRIST_KP, WristConstants.WRIST_KI, WristConstants.WRIST_KD);
        m_wristPID.setTolerance(5);

        wristSubsystemTab = Shuffleboard.getTab("WristSubsystem");

        addShuffleboardData();
    }

    private void addShuffleboardData() {
        // Booleans
        // Misc.
        wristAtSetpointEntry = wristSubsystemTab.add("At setpoint", atSetpoint()).getEntry();
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
        wristMotorOutputEntry = wristSubsystemTab.add("Motor output", m_wristMotor.getAppliedOutput()).getEntry();
        intakeMotorOutputEntry = wristSubsystemTab.add("Intake motor output", m_intakeMotor.getAppliedOutput()).getEntry();
    }

    private void updateShuffleboardData() {
        // Booleans
        // Misc.
        wristAtSetpointEntry.setBoolean(atSetpoint());
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
        wristMotorOutputEntry.setDouble(m_wristMotor.getAppliedOutput());
        intakeMotorOutputEntry.setDouble(m_intakeMotor.getAppliedOutput());
    }

    @Override
    public void periodic() {
        updateInputs(m_input);
        Logger.getInstance().processInputs("Arm Wrist", m_input);

        updateShuffleboardData();

        if (atLowerLimit()) {
            zeroWristAngle();
        }

        SmartDashboard.putNumber("Wrist Current Draw", m_wristMotor.getOutputCurrent());
        SmartDashboard.putBoolean("Intake Stalling", m_intakeMotor.getFault(CANSparkMax.FaultID.kStall));
        SmartDashboard.putBoolean("Wrist stalling", m_wristMotor.getFault(CANSparkMax.FaultID.kStall));
        SmartDashboard.putBoolean("Periodic Wrist at setpoint", atSetpoint());
    }

    public void updateInputs(WristIOInputsAutoLogged inputs){
        inputs.intakeAmps = getIntakeAmps();
        inputs.wristAngle = getWristAngle();
    }
    
    //Setters
    public void setWristAngle(double targetAngleRaw) {
        double currentWristAngle = getWristAngle();

        double targetAngleClamped = MathUtil.clamp(targetAngleRaw, Constants.LimitConstants.WRIST_SCORE_LOWER.getValue(), Constants.LimitConstants.WRIST_SCORE_UPPER.getValue());
        double targetAnglePID = MathUtil.clamp(m_wristPID.calculate(currentWristAngle, targetAngleClamped), -0.7, 0.7);

        // Dashboard variables
        prevSetpointRaw = targetAngleRaw;
        prevSetpointClamped = targetAngleClamped;
        prevSetpointPID = targetAnglePID;

        m_wristMotor.set(targetAnglePID);
    }

    static final String HITTING_SOFT_LIMIT_STRING = "Wrist Hitting Soft Limit";
    public void setWristPower(double speed) {

        if (atLowerLimit() && speed <= 0){
            m_wristMotor.set(0.0);
            SmartDashboard.putBoolean(HITTING_SOFT_LIMIT_STRING, true);
        } else if (getWristAngle() >= Constants.LimitConstants.WRIST_SCORE_UPPER.getValue() && speed >= 0) {
            SmartDashboard.putBoolean(HITTING_SOFT_LIMIT_STRING, true);
            m_wristMotor.set(0.0);
        } else {
            m_wristMotor.set(speed  * 0.2);
            SmartDashboard.putBoolean(HITTING_SOFT_LIMIT_STRING, false);
        }
    }

    public Command setWristPowerFactory(double speed) {
        if (speed == 0) {
            return runOnce(() -> setWristPower(0.1));
        } else {
            return runOnce(() -> setWristPower(speed));
        }
    }

    public void setIntakeSpeed(double speed) {
        if (speed == 0) {
            m_intakeMotor.set(0.1);
        } else {
            m_intakeMotor.set(speed * 0.2);
        }
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

    public void toggleBrakeMode() {
        if (m_wristMotor.getIdleMode() == CANSparkMax.IdleMode.kBrake) {
            m_wristMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        } else {
            m_wristMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        }
    }


    public double getIntakeAmps() {
        return m_intakeMotor.getOutputCurrent();
    }

    public boolean atLowerLimit() {
        return !m_wristZeroLimit.get();
    }

    public boolean pieceInside() {
        return m_intakeMotor.getFault(CANSparkMax.FaultID.kStall);
    }

    public boolean atSetpoint() {
        return m_wristPID.atSetpoint();
    }

    public boolean isStalling() {
        return m_intakeMotor.getFault(CANSparkMax.FaultID.kStall);
    }

    public boolean hasWristHomed() { return m_hasWristHomed; }

    public void resetHomed() {
        m_hasWristHomed = false;
    }
    public void goWristToHome() {
        if(!hasWristHomed()) {
            if(atLowerLimit()) {
                setWristPower(0);
                m_hasWristHomed = true;
            } else {
                m_wristMotor.set(-0.2);
            }
        }
    }

    public void setBrakeMode(CANSparkMax.IdleMode brakeMode) {
        m_wristMotor.setIdleMode(brakeMode);
    }

}
