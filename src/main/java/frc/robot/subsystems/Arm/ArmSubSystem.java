package frc.robot.subsystems.Arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubSystem extends SubsystemBase {
    private ArmIO m_io;
    private ArmIOInputsAutoLogged m_input;

    public final double kReverseLimitDegrees = 10;
    public final double kForwardLimitDegrees = 270;

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

    public void periodic() {
        m_io.updateInputs(m_input);
        Logger.getInstance().processInputs("Arm", m_input);
    }

}
