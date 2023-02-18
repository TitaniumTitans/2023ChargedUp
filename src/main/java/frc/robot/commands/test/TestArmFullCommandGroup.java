package frc.robot.commands.test;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.nosupersystem.FullArmControlCommand;
import frc.robot.commands.nosupersystem.WristAngToSetpoint;
import frc.robot.subsystems.arm.ArmAngleSubsystem;
import frc.robot.subsystems.arm.ArmExtSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

public class TestArmFullCommandGroup extends ParallelCommandGroup {

    public TestArmFullCommandGroup(double armExt, double armAngle, double wristAngle,
                                   ArmAngleSubsystem armAngleSubsystem, WristSubsystem wristSubsystem, ArmExtSubsystem armExtSub) {

        super(new FullArmControlCommand(armAngleSubsystem, armExtSub, armExt, armAngle),
                new WristAngToSetpoint(wristSubsystem, wristAngle));
    }
}