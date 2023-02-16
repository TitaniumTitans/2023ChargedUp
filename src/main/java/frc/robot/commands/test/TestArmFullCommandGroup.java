package frc.robot.commands.test;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.FullArmControlCommand;
import frc.robot.commands.WristAngToSetpoint;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

public class TestArmFullCommandGroup extends ParallelCommandGroup {

    public TestArmFullCommandGroup(double armExt, double armAngle, double wristAngle,
                                   ArmSubsystem armSubsystem, WristSubsystem wristSubsystem) {

        super(new FullArmControlCommand(armSubsystem, armExt, armAngle),
                new WristAngToSetpoint(wristSubsystem, wristAngle));
    }
}