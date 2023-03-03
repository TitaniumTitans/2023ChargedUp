package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmAngleSubsystem;


public class ToggleArmBrakeModeCommand extends CommandBase {
    private final ArmAngleSubsystem armAngleSubsystem;

    public ToggleArmBrakeModeCommand(ArmAngleSubsystem armAngleSubsystem) {
        this.armAngleSubsystem = armAngleSubsystem;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.armAngleSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        armAngleSubsystem.toggleBrakeMode();
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return true;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
