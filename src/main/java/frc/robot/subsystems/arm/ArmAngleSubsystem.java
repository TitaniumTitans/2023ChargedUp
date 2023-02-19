package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimitConstants;
import frc.robot.Constants.ArmConstants;
import lib.factories.SparkMaxFactory;
import lib.utils.Utils;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class ArmAngleSubsystem extends SubsystemBase {

    // Encoders
    private final CANSparkMax m_armAngleMaster;
    private final CANSparkMax m_armAngleFollower;
    private final DutyCycleEncoder m_encoderArmAngle;
    // Misc.
    private final PIDController m_anglePID;
    private ArmAngleIOInputsAutoLogged m_inputs;
    // Logging variables
    private double prevSetpointRaw;
    private double prevSetpointClamped;
    private double prevSetpointPID;

    //Shuffleboard data
    private ShuffleboardTab armAngleTab;
    private GenericEntry armAngleAtSetpointEntry;
    private GenericEntry armAngleEncoderConnectedEntry;
    private GenericEntry armAngleMotorMasterInvertedEntry;
    private GenericEntry armAngleMotorFollowerInvertedEntry;
    private GenericEntry armAngleAtUpperLimitEntry;
    private GenericEntry armAngleAtLowerLimitEntry;
    private GenericEntry armAngleEncoderRawEntry;
    private GenericEntry armAngleRawEntry;
    private GenericEntry armAngleConvertedEntry;
    private GenericEntry armAngleTargetEntry;
    private GenericEntry armAngleSetpointClampedEntry;
    private GenericEntry armAnglePIDOutputEntry;
    private GenericEntry armAngleMasterOutputEntry;
    private GenericEntry armAngleFollowerOutputEntry;

    @AutoLog
    public static class ArmAngleIOInputs {
        public double armAngle = 0.0;
    }

    public ArmAngleSubsystem() {
        SparkMaxFactory.SparkMaxConfig config = new SparkMaxFactory.SparkMaxConfig();
        config.setFrame0Rate(10);
        m_armAngleMaster = SparkMaxFactory.Companion.createSparkMax(ArmConstants.ARM_ANGLE_ID_MASTER, config);

        config.setFrame0Rate(SparkMaxFactory.MAX_CAN_FRAME_PERIOD);
        m_armAngleFollower = SparkMaxFactory.Companion.createSparkMax(ArmConstants.ARM_ANGLE_ID_FOLLOWER, config);

        m_armAngleFollower.follow(m_armAngleMaster);
    
        m_encoderArmAngle = new DutyCycleEncoder(ArmConstants.ENCODER_PORT);
        m_encoderArmAngle.reset();
        m_encoderArmAngle.setDistancePerRotation(360);
        m_encoderArmAngle.setPositionOffset(0);

        m_anglePID = new PIDController(ArmConstants.KP_ANGLE, ArmConstants.KI_ANGLE, 0.0);
        m_anglePID.setTolerance(3);

        m_inputs = new ArmAngleIOInputsAutoLogged();

        armAngleTab = Shuffleboard.getTab("ArmAngleSubsystem");
        addShuffleboardData();
    }

    private void addShuffleboardData() {
        // Booleans
        // Misc.
        armAngleAtSetpointEntry = armAngleTab.add("At setpoint", armAngleAtSetpoint()).getEntry();
        armAngleEncoderConnectedEntry = armAngleTab.add("Encoder connected", encoderConnected()).getEntry();
        // Motor inversions
        armAngleMotorMasterInvertedEntry = armAngleTab.add("Master inverted", m_armAngleMaster.getInverted()).getEntry();
        armAngleMotorFollowerInvertedEntry = armAngleTab.add("Follower inverted", m_armAngleFollower.getInverted()).getEntry();
        // Limits
        armAngleAtUpperLimitEntry = armAngleTab.add("At upper limit", armAngleAtUpperLimit()).getEntry();
        armAngleAtLowerLimitEntry = armAngleTab.add("Limit switch triggered", armAngleAtLowerLimit()).getEntry();

        // Doubles
        // Angles
        armAngleEncoderRawEntry = armAngleTab.add("Encoder raw", m_encoderArmAngle.getAbsolutePosition()).getEntry();
        armAngleRawEntry = armAngleTab.add("Angle raw", m_encoderArmAngle.getAbsolutePosition() * 360).getEntry();
        armAngleConvertedEntry = armAngleTab.add("Angle converted", getArmAngle()).getEntry();
        // Targets
        armAngleTargetEntry = armAngleTab.add("Target", prevSetpointRaw).getEntry();
        armAngleSetpointClampedEntry = armAngleTab.add("Clamped setpoint", prevSetpointClamped).getEntry();
        armAnglePIDOutputEntry = armAngleTab.add("PID setpoint output", prevSetpointPID).getEntry();
        // Misc.
        armAngleMasterOutputEntry = armAngleTab.add("Master output", m_armAngleMaster.getAppliedOutput()).getEntry();
        armAngleFollowerOutputEntry = armAngleTab.add("Follower output", m_armAngleFollower.getAppliedOutput()).getEntry();
    }

    private void updateShuffleboardData() {
        // Booleans
        // Misc.
        armAngleAtSetpointEntry.setBoolean(armAngleAtSetpoint());
        armAngleEncoderConnectedEntry.setBoolean(encoderConnected());
        // Motor inversions
        armAngleMotorMasterInvertedEntry.setBoolean(m_armAngleMaster.getInverted());
        armAngleMotorFollowerInvertedEntry.setBoolean(m_armAngleFollower.getInverted());
        // Limits
        armAngleAtUpperLimitEntry.setBoolean(armAngleAtUpperLimit());
        armAngleAtLowerLimitEntry.setBoolean(armAngleAtLowerLimit());

        // Doubles
        // Angles
        armAngleEncoderRawEntry.setDouble(m_encoderArmAngle.getAbsolutePosition());
        armAngleRawEntry.setDouble(m_encoderArmAngle.getAbsolutePosition() * 360);
        armAngleConvertedEntry.setDouble(getArmAngle());
        // Targets
        armAngleTargetEntry.setDouble(prevSetpointRaw);
        armAngleSetpointClampedEntry.setDouble(prevSetpointClamped);
        armAnglePIDOutputEntry.setDouble(prevSetpointPID);
        // Misc.
        armAngleMasterOutputEntry.setDouble(m_armAngleMaster.getAppliedOutput());
        armAngleFollowerOutputEntry.setDouble(m_armAngleFollower.getAppliedOutput());
    }

    @Override
    public void periodic() {
        updateInputs(m_inputs);
        Logger.getInstance().processInputs("Arm Angle", m_inputs);

        updateShuffleboardData();
    }

    public void updateInputs(ArmAngleIOInputsAutoLogged inputs){
        inputs.armAngle = getArmAngle();
    }

    public void setAngleSpeed(double speed) {
        if ((getArmAngle() >= LimitConstants.ARM_ANGLE_LOWER.getValue() && speed <= 0)
            || (getArmAngle() <= LimitConstants.ARM_ANGLE_UPPER.getValue() && speed >= 0)) {
            m_armAngleMaster.set(speed);
        }
    }

    public Command setAngleSpeedFactory(double speed) {
        return runOnce(() -> setAngleSpeed(speed));
    }

    public void setArmAngle(double targetAngleRaw){
        // Get angle
        double currentArmAngle = getArmAngle();

        // Clamp target
        double targetAngleClamped = MathUtil.clamp(targetAngleRaw, LimitConstants.ARM_ANGLE_LOWER.getValue(), LimitConstants.ARM_ANGLE_UPPER.getValue());
        double targetAnglePID = MathUtil.clamp(m_anglePID.calculate(currentArmAngle, targetAngleClamped), -3, 3);

        // Update dashboard variables
        prevSetpointRaw = targetAngleRaw;
        prevSetpointClamped = targetAngleClamped;
        prevSetpointPID = targetAnglePID;

        // Set voltage based off of PID
        m_armAngleMaster.setVoltage(targetAnglePID);
    }

    public double getArmAngle() {
        return Utils.normalize(360 - ((m_encoderArmAngle.getAbsolutePosition() * 360)) - ArmConstants.ARM_OFFSET);
    }

    public boolean encoderConnected() {
        return m_encoderArmAngle.isConnected();
    }

    public boolean armAngleAtSetpoint() {
        return m_anglePID.atSetpoint();
    }

    public boolean armAngleAtUpperLimit(){
        return (getArmAngle() >= LimitConstants.ARM_ANGLE_UPPER.getValue());
    }

    public boolean armAngleAtLowerLimit(){
        return (getArmAngle() <= LimitConstants.ARM_ANGLE_LOWER.getValue());
    }

    public void toggleBrakeMode() {
        if(m_armAngleMaster.getIdleMode() == CANSparkMax.IdleMode.kBrake) {
            m_armAngleMaster.setIdleMode(CANSparkMax.IdleMode.kCoast);
            m_armAngleFollower.setIdleMode(CANSparkMax.IdleMode.kCoast);
        } else {
            m_armAngleMaster.setIdleMode(CANSparkMax.IdleMode.kBrake);
            m_armAngleFollower.setIdleMode(CANSparkMax.IdleMode.kBrake);
        }
    }
}
