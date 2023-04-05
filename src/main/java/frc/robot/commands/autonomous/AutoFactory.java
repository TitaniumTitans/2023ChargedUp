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
        SINGE_SCORE,
        DOUBLE_SCORE,
        SINGLE_ENGAGE,
        DOUBLE_ENGAGE,
        TRIPLE_CONE,
        SINGLE_SCORE_ONLY
    }

    private final SwerveDrivetrain m_swerve;
    private final ArmSupersystem m_super;
    private final WristSubsystem m_wrist;
    private final ArmExtSubsystem m_armEx;

    public AutoFactory(ArmSupersystem supersystem, SwerveDrivetrain drive, WristSubsystem wrist, ArmExtSubsystem armEx) {
        this.m_super = supersystem;
        this.m_swerve = drive;
        this.m_wrist = wrist;
        this.m_armEx = armEx;
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
        Command chosenCommand;
        m_wrist.resetHomed();
        m_armEx.resetHomed();
        m_wrist.setIntakeSpeed(0.0);

        Command homeChecker = new CheckHomedCommand(m_armEx, m_wrist);

        switch (mode) {
            case MOBILITY:
                chosenCommand = new MobilityCommandGroup(m_swerve, start);
                break;
            case ENGAGE:
                chosenCommand = new BalanceCommandGroup(m_swerve, start);
                break;
            case SINGE_SCORE:
                chosenCommand = new OneCubeCommandGroup(m_super, m_swerve, m_wrist, start, height);
                break;
            case SINGLE_ENGAGE:
                chosenCommand = new ScoreOneEngageCommandGroup(m_swerve, m_super, m_wrist, start, height);
                break;
            case DOUBLE_SCORE:
                chosenCommand = new ScoreTwoCommandGroup(m_swerve, m_super, height, start);
                break;
            case DOUBLE_ENGAGE:
                chosenCommand = new ScoreTwoBalanceCommandGroup(m_swerve, m_super, height, start);
                break;
            // case TRIPLE_CONE:
                // chosenCommand = new ScoreThreeCommandGroup(m_swerve, m_super, height, start);
                // break;
            case SINGLE_SCORE_ONLY:
                chosenCommand = new ScoreOneOnlyCommandGroup(m_super, m_wrist, height);
                break;
            default:
                chosenCommand = new InstantCommand();
        }

        return homeChecker.andThen(chosenCommand);
    }
}
