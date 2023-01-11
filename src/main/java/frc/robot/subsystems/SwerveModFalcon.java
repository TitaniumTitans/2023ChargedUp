package frc.robot.subsystems;

import frc.robot.Constants.ModuleConstants;
import lib.utils.Utils;
import lib.utils.Swerve.CTREModuleState;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModFalcon {
    private TalonFX driveFx;
    private TalonFX azimuthFx;
    private CANCoder encoder;
    private double lastAngle;

    // private double cruiseVelocity = 4000.0;
    // private double maxAcceleration = 4000.0;

    public int moduleNumber;
    public double magnetOffset;
    private int count = 0;

    public SwerveModFalcon(int moduleNumber, double magnetOffset, int[] canIds){
        this.magnetOffset = magnetOffset;
        this.moduleNumber = moduleNumber;

        driveFx = new TalonFX(canIds[0]);
        azimuthFx = new TalonFX(canIds[1]);
        encoder = new CANCoder(canIds[2]);

        driveFx.setNeutralMode(NeutralMode.Brake);
        driveFx.configOpenloopRamp(0.75);

        azimuthFx.configFactoryDefault();
        azimuthFx.config_kP(0, ModuleConstants.kModuleKp);
        azimuthFx.config_kD(0, ModuleConstants.kModuleKd);
        azimuthFx.setNeutralMode(NeutralMode.Brake);
        azimuthFx.setInverted(false);
        azimuthFx.setSensorPhase(false);
        azimuthFx.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
        
        encoder.configFactoryDefault();
        encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        encoder.configSensorDirection(true);
        encoder.configMagnetOffset(magnetOffset);
        
        resetToAbsolute();
    }

    public void setDesiredState(SwerveModuleState state){
        SwerveModuleState desiredState = CTREModuleState.optimize(state, Rotation2d.fromDegrees(getAzimuthAngle()));
        // SwerveModuleState desiredState = state; //SwerveModuleState.optimize(state, getCanCoder());

        double percentOutput = desiredState.speedMetersPerSecond / 1.0 * 0.2; //This is swerve max speed , figure ths out
        

        double angle = Utils.degreesToFalcon(desiredState.angle.getDegrees(), ModuleConstants.kTurningRatio); 

        if(desiredState.speedMetersPerSecond <= 0.1 && count == 200){
            resetToAbsolute();
        }

        if(desiredState.speedMetersPerSecond > 0.1){        
            azimuthFx.set(ControlMode.Position, angle); 
        }
        driveFx.set(ControlMode.PercentOutput, percentOutput);
        lastAngle = angle;
    }

    public SwerveModuleState getState() {
        double velocity = Utils.falconToMPS(driveFx.getSelectedSensorVelocity(), ModuleConstants.kWheelCircumference, ModuleConstants.kDriveRatio);
        Rotation2d angle = Rotation2d.fromDegrees(Utils.falconToDegrees(azimuthFx.getSelectedSensorPosition(), ModuleConstants.kTurningRatio));
        return new SwerveModuleState(velocity, angle);
    }

    /*
    public SwerveModulePosition getPosition() {
        double distance = (driveFx.getSelectedSensorPosition() / 4096) * Constants.kWheelCircumfrance;
        return new SwerveModulePosition(distance, getCanCoder());
    }
    */

    public void resetToAbsolute(){
        double absolutePosition = Utils.degreesToFalcon(encoder.getAbsolutePosition(), ModuleConstants.kTurningRatio);
        azimuthFx.setSelectedSensorPosition(absolutePosition);  
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(encoder.getAbsolutePosition());
    }

    public double getAzimuthAngle(){
        return Utils.falconToDegrees(azimuthFx.getSelectedSensorPosition(), ModuleConstants.kTurningRatio);
    }

    public double getTargetAngle() {
        return lastAngle;
    }
}
