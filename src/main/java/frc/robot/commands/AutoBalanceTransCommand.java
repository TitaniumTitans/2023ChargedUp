package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDrivetrain;


public class AutoBalanceTransCommand extends CommandBase {
    private final SwerveDrivetrain m_swerve;
    private final PIDController m_pid;
    Translation2d tilt;
    public AutoBalanceTransCommand(SwerveDrivetrain swerve) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        m_swerve = swerve;
        m_pid = new PIDController(0.003, 0.0, 0.0);
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        m_pid.setSetpoint(0.0);
        m_swerve.setFieldRelative(false);
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Roll", getRoll());
        SmartDashboard.putNumber("Pitch", getPitch());

        tilt = new Translation2d(getPitch(), getRoll());

        double output = m_pid.calculate(tilt.getNorm());
        SmartDashboard.putNumber("Norm", tilt.getNorm());

        m_swerve.drive(-tilt.times(output).getX(), tilt.times(output).getY(), 0.0);
    }

    @Override
    public boolean isFinished() {
        return tilt.getNorm() < 2;
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
