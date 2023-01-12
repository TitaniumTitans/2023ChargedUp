package frc.robot.subsystems;

//import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;;

public class SwerveDrivetrain extends SubsystemBase{
  private SwerveModFalcon m_frModule;
  private SwerveModFalcon m_flModule;
  private SwerveModFalcon m_brModule;
  private SwerveModFalcon m_blModule;
  private PigeonIMU m_gyro;

  private int counter;
  private boolean fieldRelative;

  private SwerveDriveOdometry m_odometry;
  /** Creates a new ExampleSubsystem. */
  public SwerveDrivetrain() {
    m_frModule = new SwerveModFalcon(0, DriveConstants.kModFrOffset, DriveConstants.kMod0Cans);
    m_flModule = new SwerveModFalcon(1, DriveConstants.kModFlOffset, DriveConstants.kMod1Cans);
    m_brModule = new SwerveModFalcon(2, DriveConstants.kModBrOffset, DriveConstants.kMod2Cans);
    m_blModule = new SwerveModFalcon(3, DriveConstants.kModBlOffset, DriveConstants.kMod3Cans);

    m_gyro = new PigeonIMU(15);
    m_odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getGyro(), getModulePositions());
    counter = 0;
    fieldRelative = false;
  }

  

  // Getter Methods here
  //////////////////////////////////////////////////////////////////////
  public Rotation2d getGyro(){
    return Rotation2d.fromDegrees(m_gyro.getYaw());
  }

  /**
   * Gets the current state of each module as an array of
   * SwerveModulePositions, going from Fr, Fl, Br, Bl modules
   * @return A SwerveModulePosition
   */
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
  public void setModuleState(double xTranslation, double yTranslation, double zRotation, boolean fieldRelative){
    //Converts controller inputs to working chassis speeds, to working swerve module state array
    SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates( 
      fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
      xTranslation, 
      yTranslation, 
      zRotation, 
      getGyro()
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
  public void setAbsoluteAngles(){
    m_frModule.resetToAbsolute();
    m_flModule.resetToAbsolute();
    m_blModule.resetToAbsolute();
    m_brModule.resetToAbsolute();
  }

  public void resetGyro(){
    m_gyro.setYaw(0);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase resetGyroBase() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
          resetGyro();
        });
  }

  public CommandBase toggleFieldRelative(){
    return runOnce(
      () -> {
      fieldRelative = !fieldRelative;
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(getGyro(), getModulePositions());

    SmartDashboard.putNumber("Fr Azimuth", m_frModule.getAzimuthAngle());
    SmartDashboard.putNumber("Fl Azimuth", m_flModule.getAzimuthAngle());
    SmartDashboard.putNumber("Br Azimuth", m_brModule.getAzimuthAngle());
    SmartDashboard.putNumber("Bl Azimuth", m_blModule.getAzimuthAngle());

    SmartDashboard.putNumber("Fr Setpoint", m_frModule.getTargetAngle());
    SmartDashboard.putNumber("Fl Setpoint", m_flModule.getTargetAngle());
    SmartDashboard.putNumber("Br Setpoint", m_brModule.getTargetAngle());
    SmartDashboard.putNumber("Bl Setpoint", m_blModule.getTargetAngle());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
