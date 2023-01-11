package frc.robot.subsystems;

//import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
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
  private SwerveModFalcon frModule;
  private SwerveModFalcon flModule;
  private SwerveModFalcon brModule;
  private SwerveModFalcon blModule;
  private PigeonIMU gyro;
  private int counter;

  private SwerveDriveOdometry odometry;
  /** Creates a new ExampleSubsystem. */
  public SwerveDrivetrain() {
    frModule = new SwerveModFalcon(0, DriveConstants.kModFrOffset, DriveConstants.kMod0Cans);
    flModule = new SwerveModFalcon(1, DriveConstants.kModFlOffset, DriveConstants.kMod1Cans);
    brModule = new SwerveModFalcon(2, DriveConstants.kModBrOffset, DriveConstants.kMod2Cans);
    blModule = new SwerveModFalcon(3, DriveConstants.kModBlOffset, DriveConstants.kMod3Cans);

    gyro = new PigeonIMU(15);
    odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getGyro(), getModulePositions());
    counter = 0;
  }

  

  // Getter Methods here
  //////////////////////////////////////////////////////////////////////
  public Rotation2d getGyro(){
    return Rotation2d.fromDegrees(gyro.getYaw());
  }

  
  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] modulePositions = {new SwerveModulePosition(), 
      new SwerveModulePosition(), 
      new SwerveModulePosition(), 
      new SwerveModulePosition()};

    modulePositions[0] = frModule.getPosition();
    modulePositions[1] = flModule.getPosition();
    modulePositions[2] = brModule.getPosition();
    modulePositions[3] = blModule.getPosition();

    return modulePositions;
  }

  /** Gets the current state of each module as an array of 
   * SwerveModuleState objects, going from Fr, Fl, Br, Bl modules
   * @return An SwerveModuleState array
   */
  public SwerveModuleState[] getModuleStates(){
    SwerveModuleState[] moduleStates = {new SwerveModuleState(),
      new SwerveModuleState(),
      new SwerveModuleState(),
      new SwerveModuleState()};

      moduleStates[0] = frModule.getState();
      moduleStates[1] = flModule.getState();
      moduleStates[2] = brModule.getState();
      moduleStates[3] = blModule.getState();

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
      frModule.setDesiredState(swerveModuleStates[0]);
      flModule.setDesiredState(swerveModuleStates[1]);
      blModule.setDesiredState(swerveModuleStates[2]);
      brModule.setDesiredState(swerveModuleStates[3]);

      //Counter + logic for resetting to absolute
      counter++;
      if(counter == 200){
        setAbsoluteAngles();
        counter = 0;
      }
  }
  public void setAbsoluteAngles(){
    frModule.resetToAbsolute();
    flModule.resetToAbsolute();
    blModule.resetToAbsolute();
    brModule.resetToAbsolute();
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase resetGyro() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(getGyro(), getModulePositions());

    SmartDashboard.putNumber("Fr Azimuth", frModule.getAzimuthAngle());
    SmartDashboard.putNumber("Fl Azimuth", flModule.getAzimuthAngle());
    SmartDashboard.putNumber("Br Azimuth", brModule.getAzimuthAngle());
    SmartDashboard.putNumber("Bl Azimuth", blModule.getAzimuthAngle());

    SmartDashboard.putNumber("Fr Setpoint", frModule.getTargetAngle());
    SmartDashboard.putNumber("Fl Setpoint", flModule.getTargetAngle());
    SmartDashboard.putNumber("Br Setpoint", brModule.getTargetAngle());
    SmartDashboard.putNumber("Bl Setpoint", blModule.getTargetAngle());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
