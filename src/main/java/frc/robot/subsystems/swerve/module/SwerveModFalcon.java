package frc.robot.subsystems.swerve.module;

import frc.robot.Constants.ModuleConstants;
import lib.utils.Utils;
import lib.utils.Swerve.CTREModuleState;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModFalcon {
    // Physical motors/sensors
    private final TalonFX m_driveFx;
    private final TalonFX m_azimuthFx;
    private final CANCoder m_encoder;

    // Variables stored in code
    private double m_lastAngle;
    public final double m_magnetOffset;

    public SwerveModFalcon(double magnetOffset, int[] canIds) {
        this.m_magnetOffset = magnetOffset;

        m_driveFx = new TalonFX(canIds[0]);
        m_azimuthFx = new TalonFX(canIds[1]);
        m_encoder = new CANCoder(canIds[2]);

        // Drive motor config
        m_driveFx.setNeutralMode(NeutralMode.Brake);
        m_driveFx.configOpenloopRamp(0.75);
        m_driveFx.configNeutralDeadband(0.01);

        // Azimuth/turning motor config
        m_azimuthFx.configFactoryDefault();
        m_azimuthFx.config_kP(0, ModuleConstants.MODULE_KP);
        m_azimuthFx.config_kD(0, ModuleConstants.MODULE_KD);
        m_azimuthFx.setNeutralMode(NeutralMode.Brake);
        m_azimuthFx.setInverted(false);
        m_azimuthFx.setSensorPhase(false);
        m_azimuthFx.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
        m_azimuthFx.configNeutralDeadband(0.01);
        
        // Encoder configuration
        m_encoder.configFactoryDefault();
        m_encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        m_encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        m_encoder.configSensorDirection(true);
        m_encoder.configMagnetOffset(magnetOffset);
        
        // Update azimuth encoder to match absolute value encoder
        resetToAbsolute();
    }

    /**
     * Sets the module to match a desired state
     * @param state the desired state of the module
     */
    public void setDesiredState(SwerveModuleState state) {
        SwerveModuleState desiredState = CTREModuleState.optimize(state, Rotation2d.fromDegrees(getAzimuthAngle()));
        // double desiredSpeed = desiredState.speedMetersPerSecond;
        double percentOutput = desiredState.speedMetersPerSecond; /// ModuleConstants.kMaxSpeedMetersPerSecond; //This is swerve max speed , figure ths out

        SmartDashboard.putNumber("Drive Power", percentOutput);

        double angle = Utils.degreesToFalcon(desiredState.angle.getDegrees(), ModuleConstants.TURNING_RATIO); 

        // Check to see if the module is actually moving, helps prevent additional jittering
        if(Math.abs(desiredState.speedMetersPerSecond) > 0.01){        
            m_azimuthFx.set(ControlMode.Position, angle); 
        }
        m_driveFx.set(ControlMode.PercentOutput, percentOutput);
        m_lastAngle = angle;
    }

    /**
     * Gets the current state of the module as a SwerveModuleState object
     * @return the current state of the module
     */
    public SwerveModuleState getState() {
        double velocity = Utils.falconToMPS(m_driveFx.getSelectedSensorVelocity(), ModuleConstants.WHEEL_CIRCUMFERENCE_METERS, ModuleConstants.DRIVE_RATIO);
        Rotation2d angle = Rotation2d.fromDegrees(Utils.falconToDegrees(m_azimuthFx.getSelectedSensorPosition(), ModuleConstants.TURNING_RATIO));
        return new SwerveModuleState(velocity, angle);
    }

    /**
     * Gets the current state of the module as a SwerveModulePosition object
     * @return the current state of the object
     */
    public SwerveModulePosition getPosition() {
        double distance = (m_driveFx.getSelectedSensorPosition() / 4096) * ModuleConstants.WHEEL_CIRCUMFERENCE_METERS;
        return new SwerveModulePosition(distance, getCanCoder());
    }
    
    /**
     * Resets the encoder in the Azimuth motor to match the absolute value read by the absolute encoder
     */
    public void resetToAbsolute() {
        double absolutePosition = Utils.degreesToFalcon(m_encoder.getAbsolutePosition(), ModuleConstants.TURNING_RATIO);
        m_azimuthFx.setSelectedSensorPosition(absolutePosition);  
    }

    /**
     * Gets the current reading of the absolute encoder 
     * @return the rotation of the absolute encoder as a Rotation2d
     */
    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(m_encoder.getAbsolutePosition());
    }

    /**
     * Gets the reading of the Azimuth motor's encoder
     * @return current reading of the Azimuth motor's rotation
     */
    public double getAzimuthAngle() {
        return Utils.falconToDegrees(m_azimuthFx.getSelectedSensorPosition(), ModuleConstants.TURNING_RATIO);
    }

    /**
     * Get's the setpoint angle for the module
     * @return the current setpoint angle
     */
    public double getTargetAngle() {
        return m_lastAngle;
    }
}
