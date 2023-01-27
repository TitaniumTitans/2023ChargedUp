package frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubSystem extends SubsystemBase {
    private ArmIO m_io;
    private ArmIOInputsAutoLogged m_input;


    public ArmSubSystem(ArmIO io){
        m_io = io;

        m_input = new ArmIOInputsAutoLogged();
    }

    public void setAngleSpeed() {
        m_io.setAngleSpeed(getArmAngle());
    }

    public void setArmSpeed() {
        m_io.setArmSpeed(getArmExtension());
    }

    public double getArmExtension() {
        return m_io.getArmExtension();
    }

    public double getArmAngle() {
        return m_io.getArmAngle();
    }

    public void periodic() {
        m_io.updateInputs(m_input);
    }

}
