package frc.robot.subsystems.wrist;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.*;
import lib.factories.SparkMaxFactory;

public class WristIOPhysical implements WristIO{
    private CANSparkMax m_wrist;
    private CANSparkMax m_intake;
    private PIDController m_pid;

    private DigitalInput m_lowerLimit;

    private WristIOInputsAutoLogged m_inputs;
    public WristIOPhysical() {
        // config for wrist
        SparkMaxFactory.SparkMaxConfig config = new SparkMaxFactory.SparkMaxConfig();
        config.setInverted(true);
        config.setIdleMode(CANSparkMax.IdleMode.kBrake);
        config.setCurrentLimit(20);

        m_wrist = SparkMaxFactory.Companion.createSparkMax(WristConstants.WRIST_ID, config);
        m_wrist.setOpenLoopRampRate(0.1);

        m_pid = new PIDController(WristConstants.WRIST_KP, WristConstants.WRIST_KI, WristConstants.WRIST_KD);

        // config for intake
        config.setCurrentLimit(40);
        config.setInverted(false);
        m_intake = SparkMaxFactory.Companion.createSparkMax(WristConstants.INTAKE_ID, config);

        m_inputs = new WristIOInputsAutoLogged();

        // Sensors for limiting and piece detection
        m_lowerLimit = new DigitalInput(WristConstants.LIMIT_SWITCH_PORT);
    }

    @Override
    public void updateInputs(WristIOInputsAutoLogged inputs) {
        inputs.intakeAmpOutput = m_intake.getOutputCurrent();
        inputs.intakeStalling = m_intake.getFault(CANSparkMax.FaultID.kStall);
        inputs.intakeTemperature = m_intake.getMotorTemperature();

        inputs.wristAmpOutput = m_wrist.getOutputCurrent();
        inputs.wristAtSetpoint = m_pid.atSetpoint();
        inputs.wristAtLowerLimit = m_lowerLimit.get();
    }
}
