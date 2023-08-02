package frc.robot.subsystems.wrist;


import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class WristSubsystem extends SubsystemBase {
    WristIO m_io;
    public WristIOInputsAutoLogged m_inputs;

    Mechanism2d m_mechanism;
    MechanismRoot2d m_root;
    MechanismLigament2d m_wristLigament;
    public WristSubsystem(WristIO io) {
        m_io = io;
        m_inputs = new WristIOInputsAutoLogged();

        m_mechanism = new Mechanism2d(3, 3, new Color8Bit(Color.kGray));
        m_root = m_mechanism.getRoot("Wrist", 1.5, 0.5);
        m_wristLigament = m_root.append(new MechanismLigament2d("Wrist", 2, 0, 1.5, new Color8Bit(Color.kFirstRed)));

        SmartDashboard.putData("Wrist Mechanism", m_mechanism);
    }

    @Override
    public void periodic() {
        m_io.update(m_inputs);
        Logger.getInstance().processInputs("Wrist", m_inputs);

        m_wristLigament.setAngle(m_inputs.wristAngle);
    }

    public void setBrakeMode(CANSparkMax.IdleMode brakeMode){
        m_io.setBrakeMode(brakeMode);
    }

    public void toggleBrakeMode() {
        m_io.toggleBrakeMode();
    }

    public void resetHomed() {
        m_io.resetHomed();
    }

    public void goToHomed() {
        m_io.goToHome();
    }

    public void zeroWristAngle() {
        m_io.zeroWrist();
    }

    public CommandBase setWristAngle(double setpoint) {
        return runOnce(() -> m_io.setWristAngle(m_inputs.wristAngle, setpoint));
    }

    public CommandBase setIntakeSpeed(double speed) {
        return run(() -> m_io.setIntakePower(speed));
    }

    public void setWristPower(double speed) {
        m_io.setWristPower(0.0);
    }
}

