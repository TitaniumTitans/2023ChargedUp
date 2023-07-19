package frc.robot.subsystems.wrist;

import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SimableCANSparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.WristConstants;
import org.snobotv2.sim_wrappers.SingleJointedArmSimWrapper;

public class WristIOSim {
    private final SimableCANSparkMax m_intake;
    private final SimableCANSparkMax m_wrist;
    private final SingleJointedArmSimWrapper m_wristSim;

    public WristIOSim() {
        m_intake = new SimableCANSparkMax(WristConstants.INTAKE_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        m_wrist = new SimableCANSparkMax(WristConstants.WRIST_ID, CANSparkMaxLowLevel.MotorType.kBrushless)
    }
}
