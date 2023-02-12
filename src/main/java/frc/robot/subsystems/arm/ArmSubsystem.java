package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    private final ArmIO m_io;
    private final ArmIOInputsAutoLogged m_input;

    public static final double REVERSE_LIMIT_DEGREES = 0;
    public static final double FORWARD_LIMIT_DEGREES = 270;

    public ArmSubsystem(ArmIO io){
        m_io = io;

        m_input = new ArmIOInputsAutoLogged();
    }

    public void setAngleSpeed(double speed) {
        m_io.setAngleSpeed(speed);
    }

    public void setArmSpeed(double speed) {
        m_io.setArmExtensionSpeed(speed);
    }

    public  void setArmExtension(double extension) { m_io.setArmExtension(extension);}
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
        return run(() -> setAngleSpeed(speed));
    }

    public CommandBase setArmExtensionSpeedFactory(double speed) {
        return run(() -> setArmSpeed(speed));
    }

    public CommandBase setArmAngleCommandFactory(double angle) {
        return run(() -> setArmAngle(angle));
    }

    public boolean armAngleAtSetpoint() {
        return m_io.armAngleAtSetpoint();
    }

    public boolean armExstensionAtSetpoint() {
        return m_io.armExstentionAtSetpoint();
    }

    @Override
    public void periodic() {
        m_io.updateInputs(m_input);
        Logger.getInstance().processInputs("Arm", m_input);
        SmartDashboard.putNumber("Arm extension", getArmExtension());
        SmartDashboard.putNumber("Arm ANGLE Encoder", getArmAngle());
        SmartDashboard.putBoolean("Encoder Connected?", m_io.encoderConnected());

        if (m_io.armAtLowerLimit()) {
            m_io.resetExstentionEncoder();
        }
    }

}
