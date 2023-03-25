package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.autonomous.AutoUtils.ScoringHeights;
import frc.robot.commands.autonomous.AutoUtils.StartingZones;
import frc.robot.subsystems.arm.ArmExtSubsystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.supersystems.ArmSupersystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;



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

    private final SwerveDrivetrain m_swerve;
    private final ArmSupersystem m_super;
    private final WristSubsystem m_wrist;
    private final ArmExtSubsystem m_ArmEx;

    public AutoFactory(ArmSupersystem m_super, SwerveDrivetrain m_drive, WristSubsystem m_wrist, ArmExtSubsystem m_armEx) {
        this.m_super = m_super;
        this.m_swerve = m_drive;
        this.m_wrist = m_wrist;
        this.m_ArmEx = m_armEx;
        m_modeChooser = new LoggedDashboardChooser<>("Auto Mode");
        m_heightChooser = new LoggedDashboardChooser<>("Scoring Height");
        m_locationChooser = new LoggedDashboardChooser<>("Starting Location");

        // Initialize dashboard choosers
        m_locationChooser.addDefaultOption("LEFT", StartingZones.LEFT);
        for (StartingZones start : StartingZones.values()) {
            if (start != StartingZones.LEFT) {
                m_locationChooser.addOption(start.toString(), start);
            }
        }

        m_heightChooser.addDefaultOption("LOW", ScoringHeights.LOW);
        for (ScoringHeights height : ScoringHeights.values()) {
            if (height != ScoringHeights.LOW) {
                m_heightChooser.addOption(height.toString(), height);
            }
        }

        m_modeChooser.addDefaultOption("Mobility", AutoMode.MOBILITY);

        for (AutoMode mode : AutoMode.values()) {
            if (mode != AutoMode.MOBILITY) {
                m_modeChooser.addOption(mode.toString(), mode);
            }
        }
    }

    public Command getAutoRoutine() {
        StartingZones start = m_locationChooser.get();
        ScoringHeights height = m_heightChooser.get();
        AutoMode mode = m_modeChooser.get();
        Command chosenMode = new InstantCommand();
        m_wrist.resetHomed();
        m_ArmEx.resetHomed();

        Command homeChecker = new CheckHomedCommand(m_ArmEx, m_wrist);

        switch (mode) {
            case MOBILITY:
                chosenMode = new MobilityCommandGroup(m_swerve, start);
                break;
            case ENGAGE:
                chosenMode = new BalanceCommandGroup(m_swerve, start);
                break;
            case SINGLE_SCORE_CONE:
                chosenMode = new OneConeCommandGroup(m_super, m_swerve, m_wrist, start, height);
                break;
            case SINGE_SCORE_CUBE:
                chosenMode = new OneCubeCommandGroup(m_super, m_swerve, m_wrist, start, height);
                break;
            case SINGLE_CONE_ENGAGE:
            case SINGLE_CUBE_ENGAGE:
                chosenMode = new ScoreOneEngageCommandGroup(m_swerve, m_super, m_wrist, start, height);
                break;
            case DOUBLE_SCORE_CONE:
            case DOUBLE_SCORE_CUBE:
                chosenMode = new ScoreTwoCommandGroup(m_swerve, m_super, height, start);
                break;
        }

        return homeChecker.andThen(chosenMode);
    }
}
