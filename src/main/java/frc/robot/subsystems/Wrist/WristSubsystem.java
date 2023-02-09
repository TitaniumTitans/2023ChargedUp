package frc.robot.subsystems.Wrist;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;

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
        return run(
            () -> {
                setWristPower(speed);
        });
    }

    public void setIntakeSpeed(double speed) {
        m_io.setIntakeSpeed(speed);
    }

    public CommandBase setIntakeSpeedFactory(double speed) {
        return run(
            () -> {
                setIntakeSpeed(speed);
        });
    }


    //getters
    public double getWristAngle() {
        return m_io.getWristAngle() / WristConstants.WRIST_PIVOT_RATIO;
    }

    public double getIntakeAmps() {
        return m_io.getIntakeAmps();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Wrist At Limit", atLimit());
        SmartDashboard.putNumber("Wrist encoder", getWristAngle());
        m_io.updateInputs(m_input);
        Logger.getInstance().processInputs("Wristy", m_input);
    }

    public boolean atLimit() {
        return m_io.atLimit();
    }
}
