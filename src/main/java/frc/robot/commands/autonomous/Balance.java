// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.AutoBalanceTransCommand;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Balance extends SequentialCommandGroup {
  /** Creates a new Balance. */
  public Balance(SwerveDrivetrain mSwerve) {
    // Add your commands in the addCommands() call, e.g.
    addCommands(new RunCommand(() ->
            mSwerve.drive(Constants.AutoConstants.MAX_VELOCITY_MPS_AUTO, 0.0, 0.0), mSwerve)
      .raceWith(new WaitCommand(1.0)));
    addCommands(new AutoBalanceTransCommand(mSwerve));
  }
}
