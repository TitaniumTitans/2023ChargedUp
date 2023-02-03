package frc.robot.subsystems.Swerve.SwerveModules;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModNeo {
  public final int moduleNumber;
  private int kTimeoutMillisecs = 200;

  private final CANSparkMax driveMotor;
  private final RelativeEncoder driveEncoder;
//   private final SparkMaxPIDController drivePID;
//   private final SimpleMotorFeedforward driveFeedforward;

  private final CANSparkMax angleMotor;
  private final RelativeEncoder angleEncoder;
  private final SparkMaxPIDController anglePID;
  
  private final CANCoder canCoder;
  private final double canCoderOffsetDegrees;

  private double lastAngle;
  private boolean invert;

  public SwerveModNeo(int moduleNumber, double offsets, int[] canIds, boolean driveInvert) {
    this.moduleNumber = moduleNumber;
    this.invert = driveInvert;
    
    driveMotor = new CANSparkMax(canIds[0], MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    // drivePID = driveMotor.getPIDController();
    // driveFeedforward = new SimpleMotorFeedforward(Constants.kSwerve.DRIVE_KS, Constants.kSwerve.DRIVE_KV, Constants.kSwerve.DRIVE_KA);

    angleMotor = new CANSparkMax(canIds[1], MotorType.kBrushless);
    angleEncoder = angleMotor.getEncoder();
    anglePID = angleMotor.getPIDController();

    canCoder = new CANCoder(canIds[2]);
    canCoderOffsetDegrees = offsets;

    configureDevices();
    lastAngle = getState().angle.getRadians();
  }

  public void setDesiredState(SwerveModuleState state) {
    // Prevents angle motor from turning further than it needs to. 
    // E.G. rotating from 10 to 270 degrees CW vs CCW.
    state = SwerveModuleState.optimize(state, getState().angle);

    if (true) {
      double speed = state.speedMetersPerSecond;
    //   drivePID.setReference(speed, CANSparkMax.ControlType.kDutyCycle);
    driveMotor.set(speed);
    } else {
    //   drivePID.setReference(state.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity, 0, driveFeedforward.calculate(state.speedMetersPerSecond));
    }

    double angle = Math.abs(state.speedMetersPerSecond) <= 0.01
      ? lastAngle
      : state.angle.getRadians();

    anglePID.setReference(angle, CANSparkMax.ControlType.kPosition);
    lastAngle = angle;
  }

  public SwerveModuleState getState() {
    double velocity = driveEncoder.getVelocity();
    Rotation2d rot = new Rotation2d(angleEncoder.getPosition());
    return new SwerveModuleState(velocity, rot);
  }

//   public double getCanCoder() {
//     return canCoder.getAbsolutePosition();
//   }

  public Rotation2d getAngle() {
    return new Rotation2d(angleEncoder.getPosition());
  }

  public SwerveModulePosition getPosition() {
    double distance = driveEncoder.getPosition();
    Rotation2d rot = new Rotation2d(angleEncoder.getPosition());
    return new SwerveModulePosition(distance, rot);
  }

  public Rotation2d getCanCoder(){
    return Rotation2d.fromDegrees(canCoder.getAbsolutePosition() - canCoderOffsetDegrees);
  }

  public double getTargetAngle(){
    return lastAngle;
  }

  public void resetToAbsolute(){
    angleEncoder.setPosition(Units.degreesToRadians(getCanCoder().getDegrees() - canCoderOffsetDegrees));
  }

  private void configureDevices() {
    // CanCoder configuration.
    CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
    canCoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    canCoderConfiguration.sensorDirection = false;
    canCoderConfiguration.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    canCoderConfiguration.sensorTimeBase = SensorTimeBase.PerSecond;
    
    canCoder.configFactoryDefault();
    canCoder.configAllSettings(canCoderConfiguration, kTimeoutMillisecs);
    canCoder.configMagnetOffset(0, kTimeoutMillisecs);

    // Drive motor configuration.
    driveMotor.restoreFactoryDefaults();
    driveMotor.setInverted(invert);
    driveMotor.setIdleMode(IdleMode.kBrake);
    driveMotor.setOpenLoopRampRate(0.5);
    // driveMotor.setClosedLoopRampRate(Constants.kSwerve.CLOSED_LOOP_RAMP);
    // driveMotor.setSmartCurrentLimit(Constants.kSwerve.DRIVE_CURRENT_LIMIT);
 
    // drivePID.setP(Constants.kSwerve.DRIVE_KP);
    // drivePID.setI(Constants.kSwerve.DRIVE_KI);
    // drivePID.setD(Constants.kSwerve.DRIVE_KD);
    // drivePID.setFF(Constants.kSwerve.DRIVE_KF);
 
    driveEncoder.setPositionConversionFactor(ModuleConstants.WHEEL_CIRCUMFERENCE_METERS / ModuleConstants.DRIVE_RATIO);
    // driveEncoder.setVelocityConversionFactor(Constants.kSwerve.DRIVE_RPM_TO_METERS_PER_SECOND);
    driveEncoder.setPosition(0);

    // Angle motor configuration.
    angleMotor.restoreFactoryDefaults();
    angleMotor.setInverted(true);
    angleMotor.setIdleMode(IdleMode.kBrake);
    angleMotor.setSmartCurrentLimit(40);

    anglePID.setP(ModuleConstants.MODULE_KP);
    anglePID.setI(0.0);
    anglePID.setD(ModuleConstants.MODULE_KD);
    anglePID.setFF(0.0);

    anglePID.setPositionPIDWrappingEnabled(true);
    anglePID.setPositionPIDWrappingMaxInput(2 * Math.PI);
    anglePID.setPositionPIDWrappingMinInput(0);

    angleEncoder.setPositionConversionFactor(ModuleConstants.POSITION_CONVERSION_FACTOR);
    // angleEncoder.setVelocityConversionFactor(Constants.kSwerve.ANGLE_RPM_TO_RADIANS_PER_SECOND);
    angleEncoder.setPosition(Units.degreesToRadians(canCoder.getAbsolutePosition() - canCoderOffsetDegrees));    
  }
}
