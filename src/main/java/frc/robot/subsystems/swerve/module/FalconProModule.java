package frc.robot.subsystems.swerve.module;

import com.ctre.phoenixpro.StatusCode;
import com.ctre.phoenixpro.configs.CANcoderConfiguration;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.PositionDutyCycle;
import com.ctre.phoenixpro.controls.VelocityVoltage;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.ctre.phoenixpro.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.ModuleConstants;

public class FalconProModule implements SwerveModuleInterface{

    private final TalonFX m_driveMotor;
    private final TalonFX m_azimuthMotor;
    private final CANcoder m_encoder;

    private int m_lastAngle;
    private double m_magnetOffset;

    private PositionDutyCycle m_azimuthControl = new PositionDutyCycle(0);
    private VelocityVoltage m_driveControl = new VelocityVoltage(0);

    public FalconProModule(double angleOffset, int[] moduleIds) {
        m_driveMotor = new TalonFX(moduleIds[0]);
        m_azimuthMotor = new TalonFX(moduleIds[1]);
        m_encoder = new CANcoder(moduleIds[2]);

        m_lastAngle = 0;
        m_magnetOffset = angleOffset;

        m_azimuthControl.EnableFOC = true;
        m_driveControl.EnableFOC = true;

        configureDevices();
        setMagnetOffset();
    }

    /**
     * Set the module to a desired state given by the swerve module kinematics
     * @param moduleState the desired state of the module
     */
    public void setModuleState(SwerveModuleState moduleState) {
        //Optimize the state so the module doesn't rotate more than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(moduleState, getAbsoluteAngle());

        // Check to see if there is actual input, if not don't move the azimuth motor
        if (state.speedMetersPerSecond > 0.1) {
            m_azimuthControl.Position = moduleState.angle.getRotations();
        }

        m_driveControl.Velocity = calculateRPSForMPS(moduleState.speedMetersPerSecond);

        // Set the motor outputs
        m_driveMotor.setControl(m_driveControl);
        m_azimuthMotor.setControl(m_azimuthControl);
    }

    /**
     * gets the current state of the module
     * azimuth angle as a Rotation2d
     * speed in MPS
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
            m_driveMotor.getDutyCycle().getValue(),
            getModuleAngle()
        );
    }

    /**
     * calculate the correct rotational speed in Rotations per second
     * for an input in meters per second
     * Also works for converting meters into rotations
     * @param metersPerSecond desired speed in meters per second
     * @return converted output speed in rotations per second
     */
    public double calculateRPSForMPS(double metersPerSecond) {
        return (metersPerSecond / (Math.PI / ModuleConstants.WHEEL_DIAMETER_METERS)) * ModuleConstants.L3_GEAR_RATIO;
    }

    public double calculateMPSForRPS(double rotationsPerSecond) {
        return (rotationsPerSecond * (Math.PI * ModuleConstants.WHEEL_DIAMETER_METERS)) / ModuleConstants.L3_GEAR_RATIO;
    }

    /**
     * Gets the position of the absolute value CANCoder
     * @return the absolute position of the azimuth wheel as a Rotation2d object
     */
    public Rotation2d getAbsoluteAngle() {
        return Rotation2d.fromRotations(m_encoder.getAbsolutePosition().getValue());
    }

    /**
     * Gets the position of the DutyCycle encoder within the azimuth motor
     * @return the read value from the azimuth motor
     */
    public Rotation2d getModuleAngle() {
        return Rotation2d.fromRotations(m_driveMotor.getDutyCycle().getValue());
    }

    /**
     * Update the position offsets of the azimuth motor off the absolute
     * encoder values
     */
    public void setMagnetOffset() {
        m_azimuthMotor.setRotorPosition(m_encoder.getPosition().getValue());
    }
    private void configureDevices() {
        //Get the configurators for each device
        var driveConfigurator = m_driveMotor.getConfigurator();
        var azimuthConfigurator = m_azimuthMotor.getConfigurator();
        var encoderConfigurator = m_encoder.getConfigurator();

        //Configure factory default
        driveConfigurator.apply(new TalonFXConfiguration());
        azimuthConfigurator.apply(new TalonFXConfiguration());
        encoderConfigurator.apply(new CANcoderConfiguration());

        // Create and apply a configuration for the drive motor
        var driveConfiguration = new TalonFXConfiguration();
        driveConfiguration.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.5;
        driveConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfiguration.MotorOutput.DutyCycleNeutralDeadband = 0.01;
        driveConfiguration.Feedback.SensorToMechanismRatio = ModuleConstants.L3_GEAR_RATIO;
        driveConfiguration.Slot0.kS = ModuleConstants.FALCON_DRIVE_KS;
        driveConfiguration.Slot0.kV = ModuleConstants.FALCON_DRIVE_KV;
        driveConfiguration.Slot0.kP = ModuleConstants.FALCON_DRIVE_KP;

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

        //repeat config until it succeeds
        do {
            returnCode = azimuthConfigurator.apply(azimuthConfiguration);
        } while(!returnCode.isOK());

        // Create and apply a configuration for the CANCoder
        var ccConfiguration = new CANcoderConfiguration();
        ccConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        ccConfiguration.MagnetSensor.MagnetOffset = m_magnetOffset;
        ccConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        do {
            returnCode = encoderConfigurator.apply(ccConfiguration);
        } while(!returnCode.isOK());
    }
}
