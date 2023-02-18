package frc.robot.commands.nosupersystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmAngleSubsystem;
import frc.robot.subsystems.arm.ArmExtSubsystem;


public class ArmExtToSetpoint extends CommandBase {
    private final ArmExtSubsystem m_armExtSubsystem;
    private final double setpoint;

    public ArmExtToSetpoint(ArmExtSubsystem armAngleSubsystem, double setpoint) {
        m_armExtSubsystem = armAngleSubsystem;
        this.setpoint = setpoint;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(m_armExtSubsystem);
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        //
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        m_armExtSubsystem.setArmExtension(setpoint);
    }

    /**
     * <p>
     * Returns whether this command has finished. Once a command finishes -- indicated by
     * this method returning true -- the scheduler will call its {@link #end(boolean)} method.
     * </p><p>
     * Returning false will result in the command never ending automatically. It may still be
     * cancelled manually or interrupted by another command. Hard coding this command to always
     * return true will result in the command executing once and finishing immediately. It is
     * recommended to use * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand}
     * for such an operation.
     * </p>
     *
     * @return whether this command has finished.
     */
    @Override
    public boolean isFinished() {
        return m_armExtSubsystem.armExstensionAtSetpoint();
    }

    /**
     * The action to take when the command ends. Called when either the command
     * finishes normally -- it is called when {@link #isFinished()} returns
     * true -- or when  it is interrupted/canceled. This is where you may want to
     * wrap up loose ends, like shutting off a motor that was being used in the command.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
        m_armExtSubsystem.setArmSpeed(0);
    }
}
