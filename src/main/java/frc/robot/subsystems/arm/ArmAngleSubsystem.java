package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
// import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
// import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import lib.utils.Utils;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class ArmAngleSubsystem extends SubsystemBase {

    private final CANSparkMax m_armAngleMaster;
    private final CANSparkMax m_armAngleFollower;

    private final DutyCycleEncoder m_encoderArmAngle;


    private final PIDController m_anglePID;

    private ArmAngleIOInputsAutoLogged m_inputs;

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
    }

    @Override
    public void periodic() {
        updateInputs(m_inputs);
        Logger.getInstance().processInputs("Arm Angle", m_inputs);
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

    public void setArmAngle(double angle){
        double currentArmAngle = getArmAngle();
        double angleSetpoint = MathUtil.clamp(angle, ArmConstants.K_REVERSE_LIMIT, ArmConstants.K_FORWARD_LIMIT);
        double pidOutput = MathUtil.clamp(m_anglePID.calculate(currentArmAngle, angleSetpoint), -6, 6);

//        SmartDashboard.putNumber("Arm Angle Setpoint", angleSetpoint);
//        SmartDashboard.putNumber("Arm Angle PID output", pidOutput);
//        SmartDashboard.putNumber("Current Arm Angle", currentArmAngle);

        m_armAngleMaster.setVoltage(-pidOutput);
    }

    public double getArmAngle() {
        double armOffset = 260.9;
        return Utils.normalize((m_encoderArmAngle.getAbsolutePosition() * 360) - armOffset);
    }

    public boolean encoderConnected() {
        return m_encoderArmAngle.isConnected();
    }

    public boolean armAngleAtSetpoint() {
        return m_anglePID.atSetpoint();
    }


}
