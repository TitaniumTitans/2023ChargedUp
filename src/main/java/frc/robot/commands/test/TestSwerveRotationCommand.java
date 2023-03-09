package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDrivetrain;


public class TestSwerveRotationCommand extends CommandBase {
    private final SwerveDrivetrain swerveDrivetrain;
    private final boolean clockwise;

    public TestSwerveRotationCommand(SwerveDrivetrain swerveDrivetrain, boolean clockwise) {
        this.swerveDrivetrain = swerveDrivetrain;
        this.clockwise = clockwise;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.swerveDrivetrain);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (clockwise) {
            swerveDrivetrain.drive(0.0, 0.0, 0.2);
        } else {
            swerveDrivetrain.drive(0.0, 0.0, -0.5);
        }
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
