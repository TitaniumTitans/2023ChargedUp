package frc.robot.commands.Test;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Wrist.WristSubsystem;

public class TestArmFullCommandGroup extends ParallelCommandGroup {

    public TestArmFullCommandGroup(double armExt, double armAngle, double wristAngle,
                                   ArmSubsystem armSubsystem, WristSubsystem wristSubsystem) {

        super(new ArmToSetpoint(armSubsystem, armAngle).andThen(new ArmExtendToSetpoint(armSubsystem, armExt)),
                new WristToSetpointCommand(wristSubsystem, wristAngle));
    }
}