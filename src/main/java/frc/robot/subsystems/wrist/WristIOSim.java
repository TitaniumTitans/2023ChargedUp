package frc.robot.subsystems.wrist;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SimableCANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.Constants.WristConstants;
import frc.robot.Robot;
import org.snobotv2.interfaces.IDigitalWrapper;
import org.snobotv2.module_wrappers.rev.RevEncoderSimWrapper;
import org.snobotv2.module_wrappers.rev.RevMotorControllerSimWrapper;
import org.snobotv2.sim_wrappers.SingleJointedArmSimWrapper;

public class WristIOSim implements WristIO{
    private final SimableCANSparkMax m_intake;
    private final SimableCANSparkMax m_wrist;
    private final SingleJointedArmSimWrapper m_wristSim;
    private final PIDController m_pid;

    private double old_setpoint = 0;

    public WristIOSim() {
        if (!Robot.isSimulation()) {
            DriverStation.reportError("Cannot use a simulation IO outside of simulation: WristSimIO", true);
            throw new RuntimeException("Cannot use a simulation IO outside of simulation: WristSimIO");
        }

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

        m_pid = new PIDController(WristConstants.WRIST_KP, WristConstants.WRIST_KI, WristConstants.WRIST_KD);
    }

    @Override
    public void update(WristIOInputsAutoLogged inputs) {
        m_wristSim.update();

        inputs.intakeAmpOutput = m_intake.getOutputCurrent();
        inputs.intakeStalling = m_intake.getFault(CANSparkMax.FaultID.kStall);
        inputs.intakeTemperature = m_intake.getMotorTemperature();

        inputs.wristAmpOutput = m_wrist.getOutputCurrent();
        inputs.wristAtSetpoint = m_pid.atSetpoint();
        inputs.wristAtLowerLimit = inputs.wristAngle <= 0.0;
        inputs.wristPidSetpoint = old_setpoint;
        inputs.wristAtUpperLimit = m_wrist.getEncoder().getPosition() > Constants.LimitConstants.WRIST_SCORE_UPPER.getValue();
        inputs.wristOutputRaw = m_wrist.getAppliedOutput();
        inputs.wristTemperature = m_wrist.getMotorTemperature();
        inputs.wristStalling = m_wrist.getFault(CANSparkMax.FaultID.kStall);
        inputs.wristAngle = m_wrist.getEncoder().getPosition();
    }

    @Override
    public void setIntakePower(double speed) {
        m_intake.set(speed);
    }

    @Override
    public void setWristAngle(double angle, double setpoint) {
        old_setpoint = setpoint;
        double pidOutput = m_pid.calculate(angle, setpoint);
        double clampedOutput = MathUtil.clamp(pidOutput, -0.7, 0.7);

        m_wrist.set(clampedOutput);
    }

    @Override
    public void setWristPower(double speed) {
        m_wrist.set(speed);
    }

    @Override
    public void setBrakeMode(CANSparkMax.IdleMode brakeMode) {}
}
