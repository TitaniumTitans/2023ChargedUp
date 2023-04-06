package frc.robot.subsystems.swerve.module;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenixpro.StatusCode;
import com.ctre.phoenixpro.configs.CANcoderConfiguration;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.controls.PositionDutyCycle;
import com.ctre.phoenixpro.controls.VelocityDutyCycle;
import com.ctre.phoenixpro.controls.VelocityVoltage;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.ctre.phoenixpro.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ModuleConstants;
import lib.utils.Swerve.CTREModuleState;
import lib.utils.Swerve.FalconProConfigFactory;
import lib.utils.Utils;

import static com.ctre.phoenix.motorcontrol.TalonFXControlMode.Velocity;

public class FalconProModule implements SwerveModuleInterface {

    private final TalonFX m_driveMotor;
    private final TalonFX m_azimuthMotor;
    private final CANCoder m_encoder;

    private int m_lastAngle;
    private double m_magnetOffset;

    private final PositionDutyCycle m_azimuthControl = new PositionDutyCycle(0);
    private final VelocityVoltage m_driveControl = new VelocityVoltage(0);

    public FalconProModule(double angleOffset, int[] moduleIds) {
        m_driveMotor = new TalonFX(moduleIds[0]);
        m_azimuthMotor = new TalonFX(moduleIds[1]);
        m_encoder = new CANCoder(moduleIds[2]);

        m_lastAngle = 0;
        m_magnetOffset = angleOffset;

        m_azimuthControl.EnableFOC = true;
        m_driveControl.EnableFOC = true;

        configureDevices();
        setMagnetOffset();
    }

    /**
     * Set the module to a desired state given by the swerve module kinematics
     *
     * @param state the desired state of the module
     */
    public void setDesiredState(SwerveModuleState state) {
        //Optimize the state so the module doesn't rotate more than 90 degrees
        state = CTREModuleState.optimize(state, getModuleAngle());

        // Check to see if there is actual input, if not don't move the azimuth motor
//        if (Math.abs(state.speedMetersPerSecond) > 0.1) {
            m_azimuthControl.Position = state.angle.getRotations();
//        }

        m_driveControl.Velocity = calculateRPSForMPS(state.speedMetersPerSecond);

        SmartDashboard.putNumber("Desired Angle", state.angle.getRotations());
        SmartDashboard.putNumber("Desired Speed MPS", state.speedMetersPerSecond);
        SmartDashboard.putNumber("Calculated speed RPS", calculateRPSForMPS(state.speedMetersPerSecond));

        // Set the motor outputs
        m_driveMotor.setControl(m_driveControl);
//        m_driveMotor.setControl(new DutyCycleOut(state.speedMetersPerSecond / ModuleConstants.MAX_SPEED_L1_MPS));
        m_azimuthMotor.setControl(m_azimuthControl);
    }

    /**
     * gets the current state of the module
     * azimuth angle as a Rotation2d
     * speed in MPS
     *
     * @return the state of the module
     */
    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(
                calculateMPSForRPS(m_driveMotor.getVelocity().getValue()),
                getModuleAngle()
        );
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
                calculateMPSForRPS(m_driveMotor.getPosition().getValue()),
                getModuleAngle()
        );
    }

    /**
     * calculate the correct rotational speed in Rotations per second
     * for an input in meters per second
     * Also works for converting meters into rotations
     *
     * @param metersPerSecond desired speed in meters per second
     * @return converted output speed in rotations per second
     */
    public double calculateRPSForMPS(double metersPerSecond) {
        return (metersPerSecond / (Math.PI * ModuleConstants.WHEEL_DIAMETER_METERS));

    }

    public double calculateMPSForRPS(double rotationsPerSecond) {
        return (rotationsPerSecond * (Math.PI * ModuleConstants.WHEEL_DIAMETER_METERS));
    }

    /**
     * Gets the position of the absolute value CANCoder
     *
     * @return the absolute position of the azimuth wheel as a Rotation2d object
     */
    public Rotation2d getAbsoluteAngle() {
        return Rotation2d.fromDegrees(m_encoder.getAbsolutePosition());
    }

    /**
     * Gets the position of the DutyCycle encoder within the azimuth motor
     *
     * @return the read value from the azimuth motor
     */
    public Rotation2d getModuleAngle() {
        return Rotation2d.fromDegrees(Utils.normalize(Units.rotationsToDegrees(m_azimuthMotor.getPosition().getValue())));
    }

    /**
     * Update the position offsets of the azimuth motor off the absolute
     * encoder values
     */
    public void setMagnetOffset() {
        m_azimuthMotor.setRotorPosition(Units.degreesToRotations(m_encoder.getAbsolutePosition() - m_magnetOffset));
    }

    private void configureDevices() {
        //Get the configurators for each device
        var driveConfigurator = m_driveMotor.getConfigurator();
        var azimuthConfigurator = m_azimuthMotor.getConfigurator();

        //Configure factory default
        driveConfigurator.apply(new TalonFXConfiguration());
        azimuthConfigurator.apply(new TalonFXConfiguration());

        // Create and apply a configuration for the drive motor
        var driveConfiguration = new TalonFXConfiguration();
        driveConfiguration.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.7;
        driveConfiguration.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.7;
        driveConfiguration.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.4;
        driveConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfiguration.MotorOutput.DutyCycleNeutralDeadband = 0.01;
        driveConfiguration.Feedback.SensorToMechanismRatio = ModuleConstants.L3_GEAR_RATIO;
        driveConfiguration.Slot0.kS = ModuleConstants.FALCON_DRIVE_KS;
        driveConfiguration.Slot0.kV = ModuleConstants.FALCON_DRIVE_KV;
        driveConfiguration.Slot0.kP = ModuleConstants.FALCON_DRIVE_KP;
        driveConfiguration.Slot0.kI = 0.0;
        driveConfiguration.Slot0.kD = 0.0;
        driveConfiguration.Audio.BeepOnBoot = true;

        // Set CAN frame limits
//        m_driveMotor.getSupplyVoltage().setUpdateFrequency(500);
//        m_driveMotor.getFault_BootDuringEnable().setUpdateFrequency(1000);
//        m_driveMotor.getFault_DeviceTemp().setUpdateFrequency(500);
//        m_driveMotor.getFault_OverSupplyV().setUpdateFrequency(100);
//        m_driveMotor.getFault_ReverseHardLimit().setUpdateFrequency(100);

        FalconProConfigFactory.setStatusFrames(m_driveMotor);
        FalconProConfigFactory.setStatusFrames(m_azimuthMotor);

        // repeat config until it succeeds
        StatusCode returnCode;
        do {
        returnCode = driveConfigurator.apply(driveConfiguration);
        } while(!returnCode.isOK());

        //Create and apply a configuration for the azimuth motor
        var azimuthConfiguration = new TalonFXConfiguration();
        azimuthConfiguration.Slot0.kP = ModuleConstants.FALCON_AZIMUTH_KP;
        azimuthConfiguration.Slot0.kI = ModuleConstants.FALCON_AZIMUTH_KI;
        azimuthConfiguration.Slot0.kD = ModuleConstants.FALCON_AZIMUTH_KD;
        azimuthConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        azimuthConfiguration.MotorOutput.Inverted = ModuleConstants.FALCON_AZIMUTH_INVERT;
        azimuthConfiguration.MotorOutput.DutyCycleNeutralDeadband = 0.01;
        azimuthConfiguration.Feedback.SensorToMechanismRatio = ModuleConstants.TURNING_RATIO;
        azimuthConfiguration.ClosedLoopGeneral.ContinuousWrap = true;
        azimuthConfigurator.setRotorPosition(Units.degreesToRotations(m_encoder.getAbsolutePosition() - m_magnetOffset));

        //repeat config until it succeeds
        do {
        returnCode = azimuthConfigurator.apply(azimuthConfiguration);
        } while(!returnCode.isOK());

        m_azimuthMotor.setRotorPosition(getAbsoluteAngle().getDegrees() - m_magnetOffset);

        // Create and apply a configuration for the CANCoder
        m_encoder.configFactoryDefault();
        m_encoder.configSensorDirection(false);
        m_encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        m_encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        m_encoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 20);

    }
}
