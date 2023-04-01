   // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import lib.factories.SparkMaxFactory;
import org.littletonrobotics.junction.*;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private PowerDistribution m_pdh;


  private Timer m_timer;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    //Base code written from the AdvantageKit logging framework (6328 Mechanical Advantage)
    //Sets up a base logger for non-subsystem inputs

    Logger logger = Logger.getInstance();
    m_pdh = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);
    m_pdh.setSwitchableChannel(false);

     // Set up data receivers & replay source
     switch (Constants.CURRENT_MODE) {
      // Running on a real robot, log to a USB stick
      case HELIOS_V2:
      case SIM:
      case HELIOS_V1:
      logger.addDataReceiver(new WPILOGWriter("/media/sda1/helios"));
      logger.addDataReceiver(new NT4Publisher());
      break;

      // Running a physics simulator, log to local folder



      // Replaying a log, set up replay source
      case REPLAY:
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        logger.setReplaySource(new WPILOGReader(logPath));
        logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    logger.start();

    Timer.delay(0.05);
    m_timer = new Timer();

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Checks every 100 milliseconds (roughly 10 robot cycles) to see if any Spark Maxes have rebooted
    // if one has it will then rerun CAN ID configurations on it to stop CAN bus from overflowing
    if (m_timer.get() >= 5.0) {
      SparkMaxFactory.Companion.updateCanFramePeriods();
      m_timer.reset();
    }

    // For testing purposes
    m_robotContainer.getArmSupersystem().calculateArmAngleLimit(20);
    m_robotContainer.getArmSupersystem().getDriveSpeed();

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_pdh.setSwitchableChannel(false);
  }

  @Override
  public void disabledPeriodic() {
    // We don't have any periodic code
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    m_pdh.setSwitchableChannel(true);

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }


  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // We don't have any periodic code
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    SparkMaxFactory.Companion.rerunConfigs();
    m_pdh.setSwitchableChannel(true);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // We don't have any periodic code

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    // We don't have any test periodic code
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    // We don't have any init code
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    // We don't have any periodic code
  }
}
