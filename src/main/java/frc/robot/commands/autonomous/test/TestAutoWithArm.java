package frc.robot.commands.autonomous.test;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.SupersystemToPoseCommand;
import frc.robot.commands.autonomous.AutoUtils;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.supersystems.ArmSupersystem;

import java.util.HashMap;

public class TestAutoWithArm {
    public static Command getAuto(SwerveDrivetrain swerve, ArmSupersystem supersystem) {
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("ArmPose", new SupersystemToPoseCommand(supersystem, Constants.ArmSetpoints.INTAKE_CONE));
        return AutoUtils.getAutoEventRoutine(PathPlanner.loadPath("TestGoForward", AutoUtils.getDefaultConstraints()),
                eventMap,
                swerve);
    }
}
