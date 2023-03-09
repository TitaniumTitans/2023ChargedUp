package frc.robot.commands.test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDrivetrain;


public class TestModuleCommand extends CommandBase {
    SwerveDrivetrain m_drive;
    int module;
    public TestModuleCommand(SwerveDrivetrain drive, int module) {
        m_drive = drive;
        this.module = module;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements();
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        SwerveModuleState states[] = {
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState()
        };

        states[module] = new SwerveModuleState(0.5, Rotation2d.fromDegrees(90));

        m_drive.setModuleStates(states);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
