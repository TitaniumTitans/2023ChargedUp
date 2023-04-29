package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDrivetrain;


public class AutoBalanceTransCommand extends CommandBase {
    private final SwerveDrivetrain m_swerve;
    private final PIDController m_pid;
    Translation2d tilt;
    private final Timer m_timer;
    public AutoBalanceTransCommand(SwerveDrivetrain swerve) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        m_swerve = swerve;
        m_pid = new PIDController(0.00372, 0.0, 0.0);
        m_timer = new Timer();
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        m_pid.setSetpoint(0.0);
        m_swerve.setFieldRelative(false);
        m_swerve.drive(-1, 0, 0);
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Roll", getRoll());
        SmartDashboard.putNumber("Pitch", getPitch());

        tilt = new Translation2d(getPitch(), getRoll());

        double output = m_pid.calculate(tilt.getNorm());
        SmartDashboard.putNumber("Norm", tilt.getNorm());

        if (Math.abs(m_swerve.getGyroPitch().getDegrees()) < 1 && m_timer.get() < 0.01) {
            tilt = new Translation2d();
            m_timer.start();
        } else if (Math.abs(m_swerve.getGyroPitch().getDegrees()) > 0.25) {
            m_timer.stop();
            m_timer.reset();
        }

        m_swerve.drive(-tilt.times(output).getX(), tilt.times(output).getY(), 0.0);
    }

    @Override
    public boolean isFinished() {
        return tilt.getNorm() < 2 && m_timer.hasElapsed(2);
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.drive(0.0, 0.0, 0.1);
        m_swerve.setFieldRelative(true);
    }

    private double getRoll() {
        return m_swerve.getGyroRoll().getDegrees();
    }

    private double getPitch() {
        return m_swerve.getGyroPitch().getDegrees();
    }
}
