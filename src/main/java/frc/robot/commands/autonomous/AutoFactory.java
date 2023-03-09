package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.autonomous.AutoUtils.ScoringHeights;
import frc.robot.commands.autonomous.AutoUtils.StartingZones;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.supersystems.ArmSupersystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.HashMap;
import java.util.Map;

public class AutoFactory {

    private final LoggedDashboardChooser<AutoMode> m_modeChooser;
    private final LoggedDashboardChooser<ScoringHeights> m_heightChooser;
    private final LoggedDashboardChooser<StartingZones> m_locationChooser;
    public enum AutoMode {
        MOBILITY,
        ENGAGE,
        SINGLE_SCORE_CONE,
        SINGE_SCORE_CUBE,
        DOUBLE_SCORE_CONE,
        DOUBLE_SCORE_CUBE,
        SINGLE_CONE_ENGAGE,
        SINGLE_CUBE_ENGAGE,
        DOUBLE_CONE_ENGAGE,
        DOUBLE_CUBE_ENGAGE,
        TRIPLE_CONE
    }

    private final HashMap<StartingZones, Map<ScoringHeights, Map<AutoMode, Command>>> m_autoOptions = new HashMap<>();

    public AutoFactory(ArmSupersystem m_super, SwerveDrivetrain m_drive, WristSubsystem m_wrist) {
        m_modeChooser = new LoggedDashboardChooser<>("Auto Mode");
        m_heightChooser = new LoggedDashboardChooser<>("Scoring Height");
        m_locationChooser = new LoggedDashboardChooser<>("Starting Location");

        for (StartingZones start : StartingZones.values()) {
            m_autoOptions.put(start, new HashMap<>());
            for (ScoringHeights height : ScoringHeights.values()) {
                m_autoOptions.get(start).put(height, new HashMap<>());
                m_autoOptions.get(start).get(height).put(AutoMode.MOBILITY, new PrintCommand(String.format("Scoring mobility with %s height and %s starting zone", height.toString(), start.toString())));
            }
        }

        // Initialize dashboard choosers
        for (StartingZones start : StartingZones.values()) {
            m_locationChooser.addOption(start.toString(), start);
        }

        for (ScoringHeights height : ScoringHeights.values()) {
            m_heightChooser.addOption(height.toString(), height);
        }

        m_modeChooser.addDefaultOption("Test", AutoMode.MOBILITY);
    }

    public Command getAutoRoutine() {
        StartingZones start = m_locationChooser.get();
        ScoringHeights height = m_heightChooser.get();
        AutoMode mode = m_modeChooser.get();

        return m_autoOptions.get(start).get(height).get(mode);
    }
}
