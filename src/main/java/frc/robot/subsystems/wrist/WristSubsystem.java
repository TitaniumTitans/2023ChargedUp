package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSubsystem extends SubsystemBase{
    private WristIO m_io;
    private WristIOInputsAutoLogged m_input;


    public WristSubsystem(WristIO io){
        m_io = io;

        m_input = new WristIOInputsAutoLogged();
    }

    //setters
    public void setWristAngle(double angle) {
        m_io.setWristAngle(angle);
    }

    public void setWristPower(double speed) {
        m_io.setWristPower(speed);
    }

    public CommandBase setWristPowerFactory(double speed) {
        return run(() -> setWristPower(speed));
    }

    public void setIntakeSpeed(double speed) {
        m_io.setIntakeSpeed(speed);
    }

    public CommandBase setIntakeSpeedFactory(double speed) {
        return run(() -> setIntakeSpeed(speed));
    }


    //getters
    public double getWristAngle() {
        return m_io.getWristAngle();
    }

    public double getIntakeAmps() {
        return m_io.getIntakeAmps();
    }

    public boolean wristAtSetpoint() {
        return m_io.wristAtSetpoint();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Wrist At Limit", atLimit());
        SmartDashboard.putNumber("Wrist encoder", getWristAngle());
        m_io.updateInputs(m_input);
        Logger.getInstance().processInputs("Wristy", m_input);

        SmartDashboard.putBoolean("Valid Target", m_io.pieceInside());
        SmartDashboard.putNumber("Sensor Range", m_io.getDetectionRange());

        if (atLimit()) {
            m_io.zeroWristAngle();
        }
    }

    public boolean atLimit() {
        return m_io.atLimit();
    }
}
