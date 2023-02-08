package frc.robot.subsystems.Arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubSystem extends SubsystemBase {
    private ArmIO m_io;
    private ArmIOInputsAutoLogged m_input;

    public final double kReverseLimitDegrees = 0;
    public final double kForwardLimitDegrees = 270;

    public double armAngle = 100;

    public static class ArmHeights{
        //Low scoring
        public static final double kLowScoringPositionDegrees = 280;

        //Middle scoring
        public static final double kMiddleScoringPositionDegrees = 220.0;

        //High scoring
        public static final double kHightScoringPositionDegrees = 160.0;
    }

    public ArmSubSystem(ArmIO io){
        m_io = io;

        m_input = new ArmIOInputsAutoLogged();
    }

    public void setAngleSpeed(double speed) {
        m_io.setAngleSpeed(speed);
    }

    public void setArmSpeed(double speed) {
        m_io.setArmSpeed(speed);
    }

    public double getArmExtension() {
        return m_io.getArmExtension();
    }

    public double getArmAngle() {
        return m_io.getArmAngle();
    }

    public void setArmAngle(double degreesToAdd) {
        m_io.setArmAngle(degreesToAdd);
    }

    public CommandBase setArmAngleSpeedFactory(double speed) {
        return run(() -> {
            setAngleSpeed(speed);
        });
    }

    public CommandBase setArmExtentionFactory(double speed) {
        return run(() -> {
            setArmSpeed(speed);
        });
    }

    public CommandBase setArmAngleCommandFactory(double angle) {
        return run(() -> {
            setArmAngle(angle);
        });
    }

    public void periodic() {
        m_io.updateInputs(m_input);
        Logger.getInstance().processInputs("Arm", m_input);
        SmartDashboard.putNumber("Arm Encoder", getArmAngle());
        SmartDashboard.putBoolean("Encoder Connected?", m_io.encoderConnected());

        // m_io.setArmAngle(armAngle);
    }

}
