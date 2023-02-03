package frc.robot.subsystems.Swerve;

//Imports
//import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Swerve.SwerveModules.SwerveModFalcon;;

public class SwerveFalconIO implements SwerveIO{
  private SwerveModFalcon m_frModule; // Front Right Wheel
  private SwerveModFalcon m_flModule; // Front Left Wheel
  private SwerveModFalcon m_brModule; // Back Right Wheel
  private SwerveModFalcon m_blModule; // Back Left Wheel
  private PigeonIMU m_gyro; // Gyro for Balancing and 

  private int counter; //Counter variable for periodically resetting encoders
  private boolean fieldRelative; //Sees if it is field oriented

  /** Creates a new ExampleSubsystem. */
  public SwerveFalconIO() {
    m_frModule = new SwerveModFalcon(0, DriveConstants.MOD_FR_OFFSET, DriveConstants.MOD_FR_CANS);
    m_flModule = new SwerveModFalcon(1, DriveConstants.MOD_FL_OFFSET, DriveConstants.MOD_FL_CANS);
    m_brModule = new SwerveModFalcon(2, DriveConstants.MOD_BL_OFFSET, DriveConstants.MOD_BL_CANS);
    m_blModule = new SwerveModFalcon(3, DriveConstants.MOD_BR_OFFSET, DriveConstants.MOD_BR_CANS);

    // m_frModule = new SwerveModCANCoder(0, DriveConstants.kModFrOffset, DriveConstants.kMod0Cans);
    // m_flModule = new SwerveModCANCoder(1, DriveConstants.kModFlOffset, DriveConstants.kMod1Cans);
    // m_brModule = new SwerveModCANCoder(2, DriveConstants.kModBrOffset, DriveConstants.kMod2Cans);
    // m_blModule = new SwerveModCANCoder(3, DriveConstants.kModBlOffset, DriveConstants.kMod3Cans);

    m_gyro = new PigeonIMU(15);
    counter = 0;
    fieldRelative = false;
  }

  

  // Getter Methods here
  //////////////////////////////////////////////////////////////////////

  /**
   * Gets the Rotation of Gyro.
   * @return Rotation of Gyro
   */
  @Override
  public Rotation2d getGyroYaw(){
    return Rotation2d.fromDegrees(m_gyro.getYaw() * -1);
  }

  /**
   * Gets the current state of each module as an array of
   * SwerveModulePositions, going from Fr, Fl, Br, Bl modules
   * @return A SwerveModulePosition
   */
  @Override
  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] modulePositions = {new SwerveModulePosition(), 
      new SwerveModulePosition(), 
      new SwerveModulePosition(), 
      new SwerveModulePosition()};

    modulePositions[0] = m_frModule.getPosition();
    modulePositions[1] = m_flModule.getPosition();
    modulePositions[2] = m_brModule.getPosition();
    modulePositions[3] = m_blModule.getPosition();

    return modulePositions;
  }

  /** 
   * Gets the current state of each module as an array of 
   * SwerveModuleState objects, going from Fr, Fl, Br, Bl modules
   * @return A SwerveModuleState array
   */
  @Override
  public SwerveModuleState[] getModuleStates(){
    SwerveModuleState[] moduleStates = {new SwerveModuleState(),
      new SwerveModuleState(),
      new SwerveModuleState(),
      new SwerveModuleState()};

      moduleStates[0] = m_frModule.getState();
      moduleStates[1] = m_flModule.getState();
      moduleStates[2] = m_brModule.getState();
      moduleStates[3] = m_blModule.getState();

      return moduleStates;
  }
  // Setter Methods here
  //////////////////////////////////////////////////////////////////////

  /**
   * 
   * @param xTranslation Forward/Backwards
   * @param yTranslation Left/Right
   * @param zRotation Rotation
   * @param fieldRelative is it field oriented
   */
  @Override
  public void setModuleStates(double xTranslation, double yTranslation, double zRotation, boolean fieldRelative){
    //Converts controller inputs to working chassis speeds, to working swerve module state array
    SwerveModuleState[] swerveModuleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates( 
      fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
      xTranslation, 
      yTranslation, 
      zRotation, 
      getGyroYaw()
  )
  : new ChassisSpeeds(
      xTranslation, 
      yTranslation, 
      zRotation));

      //Pass respective values into module objects
      m_frModule.setDesiredState(swerveModuleStates[0]);
      m_flModule.setDesiredState(swerveModuleStates[1]);
      m_blModule.setDesiredState(swerveModuleStates[2]);
      m_brModule.setDesiredState(swerveModuleStates[3]);

      //Counter + logic for resetting to absolute
      counter++;
      if(counter == 200){
        setAbsoluteAngles();
        counter = 0;
      }
      
  }

  @Override
  public void updateInputs(SwerveIOInputs inputs){
    // FR module
    inputs.frAngleDeg = m_frModule.getState().angle.getDegrees();
    inputs.frDriveSpeedMPS = m_frModule.getState().speedMetersPerSecond;

    // FL module
    inputs.flAngleDeg = m_flModule.getState().angle.getDegrees();
    inputs.flDriveSpeedMPS = m_flModule.getState().speedMetersPerSecond;

    // BL module
    inputs.blAngleDeg = m_blModule.getState().angle.getDegrees();
    inputs.blDriveSpeedMPS = m_blModule.getState().speedMetersPerSecond;

    // BR module
    inputs.brAngleDeg = m_brModule.getState().angle.getDegrees();
    inputs.brDriveSpeedMPS = m_brModule.getState().speedMetersPerSecond;

    // Gyro values
    inputs.gyroPitchDeg = m_gyro.getPitch();
    inputs.gyroYawDeg = m_gyro.getYaw();
  }

  /**
   * Matching the encoder in the Motor to the CANcoder 
   */
  @Override
  public void setAbsoluteAngles(){
    m_frModule.resetToAbsolute();
    m_flModule.resetToAbsolute();
    m_blModule.resetToAbsolute();
    m_brModule.resetToAbsolute();
  }

  /**
   * Resets the Rotation of Gyro
   */
  @Override
  public void resetGyro(){
    m_gyro.setYaw(0);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  // public CommandBase resetGyroBase() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
  //   return runOnce(
  //       () -> {
  //         /* one-time action goes here */
  //         resetGyro();
  //       });
  // }


  // public CommandBase toggleFieldRelative(){
  //   return runOnce(
  //     () -> {
  //     fieldRelative = !fieldRelative;
  //   });
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Fr Azimuth", m_frModule.getAzimuthAngle());
    SmartDashboard.putNumber("Fl Azimuth", m_flModule.getAzimuthAngle());
    SmartDashboard.putNumber("Br Azimuth", m_brModule.getAzimuthAngle());
    SmartDashboard.putNumber("Bl Azimuth", m_blModule.getAzimuthAngle());

    SmartDashboard.putNumber("Fr Setpoint", m_frModule.getTargetAngle());
    SmartDashboard.putNumber("Fl Setpoint", m_flModule.getTargetAngle());
    SmartDashboard.putNumber("Br Setpoint", m_brModule.getTargetAngle());
    SmartDashboard.putNumber("Bl Setpoint", m_blModule.getTargetAngle());

    SmartDashboard.putBoolean("Field Oriented?", fieldRelative);
  }
}
