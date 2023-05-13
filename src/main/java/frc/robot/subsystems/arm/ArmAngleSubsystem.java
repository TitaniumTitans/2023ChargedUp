package frc.robot.subsystems.arm;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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
    private final ArmAngleIOInputsAutoLogged m_inputs;
    // Logging variables
    private double prevSetpointRaw;
    private double prevSetpointClamped;
    private double prevSetpointPID;

    //Shuffleboard data
    private final ShuffleboardTab armAngleTab;
    private GenericEntry armAngleAtSetpointEntry;
    private GenericEntry armAngleAtUpperLimitEntry;
    private GenericEntry armAngleAtLowerLimitEntry;
    private GenericEntry armAngleEncoderRawEntry;
    private GenericEntry armAngleRawEntry;
    private GenericEntry armAngleConvertedEntry;
    private GenericEntry armAngleTargetEntry;
    private GenericEntry armAngleSetpointClampedEntry;
    private GenericEntry armAnglePIDOutputEntry;
    private GenericEntry armAngleMasterOutputEntry;
    private GenericEntry currentCommandOutputEntry;

    @AutoLog
    public static class ArmAngleIOInputs {
        public double armAngle = 0.0;
        public double armAnglePower = 0.0;
        public double currentDraw = 0.0;
    }

    public ArmAngleSubsystem() {
        SparkMaxFactory.SparkMaxConfig config = new SparkMaxFactory.SparkMaxConfig();
        config.setFrame0Rate(10);
        config.setCurrentLimit(40);
        m_armAngleMaster = SparkMaxFactory.Companion.createSparkMax(ArmConstants.ARM_ANGLE_ID_MASTER, config);
        m_armAngleMaster.setOpenLoopRampRate(0.2);
        m_armAngleMaster.enableVoltageCompensation(12);

        config.setFrame0Rate(SparkMaxFactory.MAX_CAN_FRAME_PERIOD);
        config.setFollowingMotor(m_armAngleMaster);
        config.setInverted(Constants.CURRENT_MODE == Constants.Mode.HELIOS_V1);
        // The SparkMaxFactory will set the motor to follow the given motor
        m_armAngleFollower = SparkMaxFactory.Companion.createSparkMax(ArmConstants.ARM_ANGLE_ID_FOLLOWER, config);
        m_armAngleFollower.enableVoltageCompensation(12);

        m_encoderArmAngle = new DutyCycleEncoder(ArmConstants.ENCODER_PORT);
        m_encoderArmAngle.reset();
        m_encoderArmAngle.setDistancePerRotation(360);
        m_encoderArmAngle.setPositionOffset(0);

        m_anglePID = new PIDController(ArmConstants.KP_ANGLE, ArmConstants.KI_ANGLE, 0.0);
        m_anglePID.setTolerance(7);

        m_inputs = new ArmAngleIOInputsAutoLogged();

        armAngleTab = Shuffleboard.getTab("ArmAngleSubsystem");
        addShuffleboardData();
    }

    private void addShuffleboardData() {
        // Booleans
        // Misc.
        armAngleAtSetpointEntry = armAngleTab.add("At setpoint", atSetpoint()).getEntry();

        // Limits
        armAngleAtUpperLimitEntry = armAngleTab.add("At upper limit", armAngleAtUpperLimit()).getEntry();
        armAngleAtLowerLimitEntry = armAngleTab.add("At lower limit", armAngleAtLowerLimit()).getEntry();

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
        armAngleMasterOutputEntry = armAngleTab.add("Arm Angle Applied Output", 0).getEntry();
        currentCommandOutputEntry = armAngleTab.add("Current Command", "No Command").getEntry();
    }

    private void updateShuffleboardData() {
        // Booleans
        // Misc.
        armAngleAtSetpointEntry.setBoolean(atSetpoint());

        // Motor inversions
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
        Command currentCommand = getCurrentCommand();
        if(currentCommand != null) {
            currentCommandOutputEntry.setString(currentCommand.getName());
        } else {
            currentCommandOutputEntry.setString("No Command");
        }
        armAngleMasterOutputEntry.setDouble(m_armAngleMaster.getAppliedOutput());
    }

    @Override
    public void periodic() {
        updateInputs(m_inputs);
        Logger.getInstance().processInputs("Arm Angle", m_inputs);

        updateShuffleboardData();
    }

    public void updateInputs(ArmAngleIOInputsAutoLogged inputs){
        inputs.armAngle = getArmAngle();
        inputs.armAnglePower = m_armAngleMaster.getAppliedOutput();
        inputs.currentDraw = m_armAngleMaster.getOutputCurrent();
    }

    public void setAngleSpeed(double speed) {
        if ((getArmAngle() >= LimitConstants.ARM_ANGLE_LOWER.getValue() && speed <= 0)
            || (getArmAngle() <= LimitConstants.ARM_ANGLE_UPPER.getValue() && speed >= 0)) {
            m_armAngleMaster.set(speed);
        } else {
            m_armAngleMaster.set(0);
        }
    }

    public Command setAngleSpeedFactory(double speed) {
        return runOnce(() -> setAngleSpeed(speed));
    }

    public void setArmAngle(double targetAngleRaw) {
        // Get angle
        double currentArmAngle = getArmAngle();
        double targetAnglePID;
        // Clamp target
        double targetAngleClamped = MathUtil.clamp(targetAngleRaw, LimitConstants.ARM_ANGLE_LOWER.getValue(), LimitConstants.ARM_ANGLE_UPPER.getValue());
        if (DriverStation.isAutonomous()) {
            targetAnglePID = MathUtil.clamp(m_anglePID.calculate(currentArmAngle, targetAngleClamped), -10, 10);
        } else {
            if (targetAngleClamped < currentArmAngle) {
                targetAnglePID = MathUtil.clamp(m_anglePID.calculate(currentArmAngle, targetAngleClamped), -7, 7);
            } else {
                targetAnglePID = MathUtil.clamp(m_anglePID.calculate(currentArmAngle, targetAngleClamped), -9, 9);
            }
        }

        // Update dashboard variables
        prevSetpointRaw = targetAngleRaw;
        prevSetpointClamped = targetAngleClamped;
        prevSetpointPID = targetAnglePID;

        SmartDashboard.putNumber("Target Angle Clamped", targetAngleClamped);
        SmartDashboard.putNumber("Raw output", m_anglePID.calculate(currentArmAngle, targetAngleClamped));
        SmartDashboard.putNumber("Angle PID Output", targetAnglePID);

        // Set voltage based off of PID
        m_armAngleMaster.setVoltage(targetAnglePID * 0.6);
    }

    public double getArmAngle() {
        return Utils.normalize(360 - (m_encoderArmAngle.getAbsolutePosition() * 360) - ArmConstants.ARM_OFFSET);
    }

    public boolean encoderConnected() {
        return m_encoderArmAngle.isConnected();
    }

    public boolean atSetpoint() {
        return m_anglePID.atSetpoint();
    }
    public double getError() { return m_anglePID.getPositionError();}

    public boolean armAngleAtUpperLimit(){
        return (getArmAngle() >= LimitConstants.ARM_ANGLE_UPPER.getValue());
    }

    public boolean armAngleAtLowerLimit(){
        return (getArmAngle() <= LimitConstants.ARM_ANGLE_LOWER.getValue());
    }

    public void toggleBrakeMode() {
        if(m_armAngleMaster.getIdleMode() == CANSparkMax.IdleMode.kBrake) {
            disableBrakeMode();
        } else {
            enableBrakeMode();
        }
    }

    public void enableBrakeMode() {
        setBrakeMode(CANSparkMax.IdleMode.kBrake);
    }

    public void disableBrakeMode() {
        setBrakeMode(CANSparkMax.IdleMode.kCoast);
    }

    public void setBrakeMode(CANSparkMax.IdleMode idleMode) {
        m_armAngleMaster.setIdleMode(idleMode);
        m_armAngleFollower.setIdleMode(idleMode);
    }

}
