package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmExtSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;


public class CheckHomedCommand extends CommandBase {
    private final ArmExtSubsystem armExtSubsystem;
    private final WristSubsystem wristSubsystem;
    Timer timer = new Timer();
    public CheckHomedCommand(ArmExtSubsystem armExtSubsystem, WristSubsystem wristSubsystem) {
        this.armExtSubsystem = armExtSubsystem;
        this.wristSubsystem = wristSubsystem;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.armExtSubsystem, this.wristSubsystem);
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        if(!wristSubsystem.hasWristHomed()) {
            wristSubsystem.goWristToHome();
        }

        if(!armExtSubsystem.hasArmHomed()) {
            armExtSubsystem.goArmToHome();
        }
    }

    @Override
    public boolean isFinished() {
        return wristSubsystem.hasWristHomed() && armExtSubsystem.hasArmHomed() || timer.hasElapsed(.75);
    }

    @Override
    public void end(boolean interrupted) {

    }
}
