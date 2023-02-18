package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
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
    private ShuffleboardTab armAngleTab;

    @AutoLog
    public static class ArmAngleIOInputs {
        public double ArmAngle = 0.0;
    }

    public ArmAngleSubsystem() {

        m_armAngleMaster = new CANSparkMax(ArmConstants.ARM_ANGLE_ID_MASTER, MotorType.kBrushless);
        m_armAngleFollower = new CANSparkMax(ArmConstants.ARM_ANGLE_ID_FOLLOWER, MotorType.kBrushless);

        m_armAngleMaster.setInverted(false);
        m_armAngleFollower.setInverted(false);

        m_armAngleMaster.setIdleMode(IdleMode.kBrake);
        m_armAngleFollower.setIdleMode(IdleMode.kBrake);

        m_armAngleFollower.follow(m_armAngleMaster);

        m_armAngleMaster.setIdleMode(IdleMode.kBrake);
        m_armAngleFollower.setIdleMode(IdleMode.kBrake);
    
        m_encoderArmAngle = new DutyCycleEncoder(ArmConstants.ENCODER_PORT);
        m_encoderArmAngle.reset();
        m_encoderArmAngle.setDistancePerRotation(360);
        m_encoderArmAngle.setPositionOffset(0);

        m_anglePID = new PIDController(ArmConstants.KP_ANGLE, ArmConstants.KI_ANGLE, 0.0);
        m_anglePID.setTolerance(3);

        m_inputs = new ArmAngleIOInputsAutoLogged();

        armAngleTab = Shuffleboard.getTab("ArmAngleSubsystem");
    }

    @Override
    public void periodic() {
        updateInputs(m_inputs);
        Logger.getInstance().processInputs("Arm Angle", m_inputs);

        // Booleans
        // Misc.
        armAngleTab.add("At setpoint", armAngleAtSetpoint());
        armAngleTab.add("Encoder connected", encoderConnected());
        // Limits
        armAngleTab.add("At upper limit", armAtUpperLimit());
        armAngleTab.add("At lower limit", armAtLowerLimit());
        // Motor inversions
        armAngleTab.add("Master inverted", m_armAngleMaster.getInverted());
        armAngleTab.add("Follower Inverted", m_armAngleFollower.getInverted());

        // Doubles
        // Angles
        armAngleTab.add("Encoder raw", m_encoderArmAngle);
        armAngleTab.add("Angle raw", m_encoderArmAngle.getAbsolutePosition() * 360);
        armAngleTab.add("Angle converted", getArmAngle());
        // Targets
        armAngleTab.add("Target", prevSetpointRaw);
        armAngleTab.add("Clamped setpoint", prevSetpointClamped);
        armAngleTab.add("PID setpoint output", prevSetpointPID);
    }

    public void updateInputs(ArmAngleIOInputsAutoLogged inputs){
        inputs.ArmAngle = getArmAngle();
    }

    public void setAngleSpeed(double speed) {
        if ((getArmAngle() <= ArmConstants.K_REVERSE_LIMIT && speed <= 0)
            || (getArmAngle() >= ArmConstants.K_FORWARD_LIMIT && speed >= 0)) {
            m_armAngleMaster.set(speed);
        }
    }

    public Command setAngleSpeedFactory(double speed) {
        return runOnce(() -> {setAngleSpeed(speed);});
    }

    public void setArmAngle(double targetAngleRaw){
        // Get angle
        double currentArmAngle = getArmAngle();

        // Clamp target
        double targetAngleClamped = MathUtil.clamp(targetAngleRaw, ArmConstants.K_REVERSE_LIMIT, ArmConstants.K_FORWARD_LIMIT);
        double targetAnglePID = MathUtil.clamp(m_anglePID.calculate(currentArmAngle, targetAngleClamped), -6, 6);

        // Update dashboard variables
        prevSetpointRaw = targetAngleRaw;
        prevSetpointClamped = targetAngleClamped;
        prevSetpointPID = targetAnglePID;

        // Set voltage based off of PID
        m_armAngleMaster.setVoltage(-targetAnglePID);
    }

    public double getArmAngle() {
        return Utils.normalize((m_encoderArmAngle.getAbsolutePosition() * 360) - ArmConstants.ARM_OFFSET);
    }

    public boolean encoderConnected() {
        return m_encoderArmAngle.isConnected();
    }

    public boolean armAngleAtSetpoint() {
        return m_anglePID.atSetpoint();
    }

    public boolean armAtUpperLimit(){
        return (getArmAngle() >= ArmConstants.K_FORWARD_LIMIT);
    }
    public boolean armAtLowerLimit(){
        return (getArmAngle() <= ArmConstants.K_REVERSE_LIMIT);
    }
}
