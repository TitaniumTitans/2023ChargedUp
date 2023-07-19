package frc.robot.subsystems.wrist;

import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SimableCANSparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.Constants.WristConstants;
import org.snobotv2.module_wrappers.rev.RevEncoderSimWrapper;
import org.snobotv2.module_wrappers.rev.RevMotorControllerSimWrapper;
import org.snobotv2.sim_wrappers.SingleJointedArmSimWrapper;

public class WristIOSim {
    private final SimableCANSparkMax m_intake;
    private final SimableCANSparkMax m_wrist;
    private final SingleJointedArmSimWrapper m_wristSim;

    public WristIOSim() {
        m_intake = new SimableCANSparkMax(WristConstants.INTAKE_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        m_wrist = new SimableCANSparkMax(WristConstants.WRIST_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

        SingleJointedArmSim wristsim = new SingleJointedArmSim(
                DCMotor.getNeo550(1),
                WristConstants.WRIST_PIVOT_RATIO,
                1,
                Units.inchesToMeters(10), // Make not a magic number later, it's used in a few places
                Math.toRadians(Constants.LimitConstants.WRIST_SCORE_LOWER.getValue()),
                Math.toRadians(Constants.LimitConstants.WRIST_SCORE_UPPER.getValue()),
                true);

        m_wristSim = new SingleJointedArmSimWrapper(
                wristsim,
                new RevMotorControllerSimWrapper(m_wrist),
                RevEncoderSimWrapper.create(m_wrist));
    }
}
