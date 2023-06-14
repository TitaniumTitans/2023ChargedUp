package frc.robot.subsystems.wrist;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants.*;
import lib.factories.SparkMaxFactory;

public class WristIOPhysical implements WristIO{
    private CANSparkMax m_wrist;
    private CANSparkMax m_intake;

    private WristIOInputsAutoLogged m_inputs;
    public WristIOPhysical() {
        // config for wrist
        SparkMaxFactory.SparkMaxConfig config = new SparkMaxFactory.SparkMaxConfig();
        config.setInverted(true);
        config.setIdleMode(CANSparkMax.IdleMode.kBrake);
        config.setCurrentLimit(20);

        m_wrist = SparkMaxFactory.Companion.createSparkMax(WristConstants.WRIST_ID, config);
        m_wrist.setOpenLoopRampRate(0.1);

        // config for intake
        config.setCurrentLimit(40);
        config.setInverted(false);
        m_intake = SparkMaxFactory.Companion.createSparkMax(WristConstants.INTAKE_ID, config);

        m_inputs = new WristIOInputsAutoLogged();
    }
}
