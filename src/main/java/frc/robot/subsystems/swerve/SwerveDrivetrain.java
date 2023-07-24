package frc.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.autonomous.AutoUtils;
import frc.robot.subsystems.swerve.module.SwerveModNeo;
import lib.LimelightHelpers;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;
import java.util.Map;

public class SwerveDrivetrain extends SubsystemBase {
    private final SwerveModNeo m_frMod;
    private final SwerveModNeo m_flMod;
    private final SwerveModNeo m_blMod;
    private final SwerveModNeo m_brMod;

    private final WPI_Pigeon2 m_gyro;
    private final TimeOfFlight m_tofSensor;

    private final SwerveIOInputsAutoLogged m_inputs;

    private boolean fieldOriented = true;

    private final Field2d m_field;
    private final SwerveDrivePoseEstimator m_poseEstimator;
    private final SwerveDrivePoseEstimator m_visionEstimator;


    private double m_currentPitch = 0;
    private double m_previousPitch = 0;
    private double m_currentTime = 0;
    private double m_prevTime = 0;
    private boolean slowmode = false;
    private double m_speedMult = 1;
    private double m_rotationMult = 1;

    @AutoLog
    public static class SwerveIOInputs {
        // Mod fr
        public double frAngleDeg = 0.0;
        public double frDriveSpeedMPS = 0.0;

        // Mod fl
        public double flAngleDeg = 0.0;
        public double flDriveSpeedMPS = 0.0;

        // Mod br
        public double brAngleDeg = 0.0;
        public double brDriveSpeedMPS = 0.0;

        // Mod bl
        public double blAngleDeg = 0.0;
        public double blDriveSpeedMPS = 0.0;

        // General robot
        public double gyroYawDeg = 0.0;
        public double gyroPitchDeg = 0.0;
    }

    public SwerveDrivetrain() {
            m_flMod = new SwerveModNeo(0, DriveConstants.MOD_FL_OFFSET, DriveConstants.MOD_FL_CANS, false);
            m_frMod = new SwerveModNeo(1, DriveConstants.MOD_FR_OFFSET, DriveConstants.MOD_FR_CANS, false);
            m_blMod = new SwerveModNeo(2, DriveConstants.MOD_BL_OFFSET, DriveConstants.MOD_BL_CANS, false);
            m_brMod = new SwerveModNeo(3, DriveConstants.MOD_BR_OFFSET, DriveConstants.MOD_BR_CANS, false);

        m_gyro = new WPI_Pigeon2(DriveConstants.GYRO_CAN);
        m_inputs = new SwerveIOInputsAutoLogged();

        m_tofSensor = new TimeOfFlight(Constants.WristConstants.TOF_PORT);
        m_tofSensor.setRangingMode(TimeOfFlight.RangingMode.Short, 10);

        m_field = new Field2d();
        SmartDashboard.putData("Field", m_field);
        m_poseEstimator = new SwerveDrivePoseEstimator(
                DriveConstants.DRIVE_KINEMATICS,
                getGyroYaw(),
                getModulePositions(),
                new Pose2d(),
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.1),
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.75, 0.75, Units.degreesToRadians(1.5))
        );

        m_visionEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.DRIVE_KINEMATICS, 
            new Rotation2d(), 
            getModulePositions(), 
            new Pose2d());

        SmartDashboard.putData("Field", m_field);
        resetGyro();
        m_visionEstimator.update(new Rotation2d(), getModulePositions());
    }

    @Override
    public void periodic() {
        updateInputs();
        Logger.getInstance().processInputs("Swerve", m_inputs);
        Logger.getInstance().recordOutput("Robot Pose", getPose());

        updatePoseEstimator();
        m_field.setRobotPose(getPose());

//        double[] angles = getAngles();
//        SmartDashboard.putNumber("Swerve Gyro Yaw", getGyroYaw().getDegrees());
//
//        SmartDashboard.putNumber("FL Angle", angles[0]);
//        SmartDashboard.putNumber("FR Angle", angles[1]);
//        SmartDashboard.putNumber("BL Angle", angles[2]);
//        SmartDashboard.putNumber("BR Angle", angles[3]);
//
//        Rotation2d[] cancoderAngles = getCancoderAngles();
//        SmartDashboard.putNumber("FL Cancoder", cancoderAngles[0].getDegrees());
//        SmartDashboard.putNumber("FR Cancoder", cancoderAngles[1].getDegrees());
//        SmartDashboard.putNumber("BL Cancoder", cancoderAngles[2].getDegrees());
//        SmartDashboard.putNumber("BR Cancoder", cancoderAngles[3].getDegrees());
//
//        SmartDashboard.putNumber("FL Actual Speed", m_flMod.getModuleState().speedMetersPerSecond);
//        SmartDashboard.putNumber("FR Actual Speed", m_frMod.getModuleState().speedMetersPerSecond);
//        SmartDashboard.putNumber("BL Actual Speed", m_blMod.getModuleState().speedMetersPerSecond);
//        SmartDashboard.putNumber("BR Actual Speed", m_blMod.getModuleState().speedMetersPerSecond);

        m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());

        m_previousPitch = m_currentPitch;
        m_currentPitch = getGyroPitch().getDegrees();

        m_prevTime = m_currentTime;
        m_currentTime = RobotController.getFPGATime();

        //getFrontCamTagID();
    }

    // Getters
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] modPos = new SwerveModulePosition[4];

        modPos[0] = m_flMod.getModulePosition();
        modPos[1] = m_frMod.getModulePosition();
        modPos[2] = m_blMod.getModulePosition();
        modPos[3] = m_brMod.getModulePosition();

        return modPos;
    }


    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];

        states[0] = m_flMod.getModuleState();
        states[1] = m_frMod.getModuleState();
        states[2] = m_blMod.getModuleState();
        states[3] = m_brMod.getModuleState();

        return states;
    }

    public Rotation2d getGyroYaw() {
        return m_gyro.getRotation2d();
    }


    // Setters
    public void drive(double xTranslation, double yTranslation, double zRotation) {
        SwerveModuleState[] states = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
                fieldOriented ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xTranslation,
                        yTranslation,
                        zRotation * m_rotationMult,
                        getGyroYaw()
                )
                        : new ChassisSpeeds(xTranslation, yTranslation, zRotation)
        );


        setModuleStates(states);
    }

    public void setModuleStates(SwerveModuleState[] states) {
       for (SwerveModuleState state : states) {
           state.speedMetersPerSecond *= m_speedMult;
       }


        m_flMod.setDesiredState(states[0]);
        m_frMod.setDesiredState(states[1]);
        m_blMod.setDesiredState(states[2]);
        m_brMod.setDesiredState(states[3]);

        SmartDashboard.putNumber("FL Desired Speed", states[0].speedMetersPerSecond);
        SmartDashboard.putNumber("FR Desired Speed", states[1].speedMetersPerSecond);
        SmartDashboard.putNumber("BL Desired Speed", states[2].speedMetersPerSecond);
        SmartDashboard.putNumber("BR Desired Speed", states[3].speedMetersPerSecond);
    }

    public void resetGyro(double heading) {
        m_gyro.setYaw(heading);
    }

    public void resetGyro() {
        resetGyro(0);
    }

    public Rotation2d getGyroPitch() {
        return Rotation2d.fromDegrees(m_gyro.getPitch());
    }

    public Rotation2d getGyroRoll() {
        return Rotation2d.fromDegrees(m_gyro.getRoll());
    }

    public double getGyroPitchRate() {
        //Account for initial boot time
        if (m_prevTime == 0) {
            m_prevTime = RobotController.getFPGATime();
        }
        // Return the rate of falling
        double fpgaElapsedTime = RobotController.getFPGATime() - m_prevTime;
        return (m_currentPitch - m_previousPitch) / fpgaElapsedTime;
    }

    public void setAbsoluteAngles() {
        m_flMod.setMagnetOffset();
        m_frMod.setMagnetOffset();
        m_blMod.setMagnetOffset();
        m_brMod.setMagnetOffset();
    }

    public void updateInputs() {
        m_inputs.flAngleDeg = m_flMod.getModuleState().angle.getDegrees();
        m_inputs.flDriveSpeedMPS = m_flMod.getModuleState().speedMetersPerSecond;

        m_inputs.frAngleDeg = m_frMod.getModuleState().angle.getDegrees();
        m_inputs.frDriveSpeedMPS = m_frMod.getModuleState().speedMetersPerSecond;

        m_inputs.blAngleDeg = m_blMod.getModuleState().angle.getDegrees();
        m_inputs.blDriveSpeedMPS = m_blMod.getModuleState().speedMetersPerSecond;

        m_inputs.brAngleDeg = m_brMod.getModuleState().angle.getDegrees();
        m_inputs.brDriveSpeedMPS = m_brMod.getModuleState().speedMetersPerSecond;

        m_inputs.gyroPitchDeg = m_gyro.getPitch();
        m_inputs.gyroYawDeg = m_gyro.getYaw();
    }

    public void updatePoseEstimator() {
        /*
         * Get swerve odometry
         */
        m_poseEstimator.update(getGyroYaw(), getModulePositions());

        if (false) {
            Pose2d botpose = LimelightHelpers.getBotPose2d("limelight");
            double latency = LimelightHelpers.getLatency_Pipeline("limelight");

            m_poseEstimator.addVisionMeasurement(botpose, latency);
        }
    }

    public void resetPose(Pose2d newPose) {
        m_poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), newPose);
    }

    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public double[] getAngles() {
        return new double[] {
                m_flMod.getModuleAngle().getDegrees(),
                m_frMod.getModuleAngle().getDegrees(),
                m_blMod.getModuleAngle().getDegrees(),
                m_brMod.getModuleAngle().getDegrees()
        };
    }

    public Rotation2d[] getCancoderAngles() {
        return new Rotation2d[] {
                m_flMod.getAbsoluteAngle(),
                m_frMod.getAbsoluteAngle(),
                m_blMod.getAbsoluteAngle(),
                m_brMod.getAbsoluteAngle(),
        };
    }

    public SwerveAutoBuilder getAutoBuilder(Map<String, Command> eventMap) {
        return new SwerveAutoBuilder(
                this::getPose,
                this::resetPose,
                DriveConstants.DRIVE_KINEMATICS,
                AutoConstants.CONSTANTS_X,
                AutoConstants.THETA_CONSTANTS,
                this::setModuleStates,
                eventMap,
                true,
                this
        );
    }

    public Command resetGyroBase() {
        return runOnce(this::resetGyro);
    }

    public void setFieldRelative(boolean relative) {
        fieldOriented = relative;
    }

    public CommandBase resetPoseBase() {
        return runOnce(() -> resetPose(new Pose2d()));
    }

    public void setSlowmode() {
        slowmode = !slowmode;
    }

    public boolean getSlowmode() {
        return slowmode;
    }

    public CommandBase setSlowmodeFactory() {
        return runOnce(this::setSlowmode);
    }

    public void setSpeedMult(double mult) {
        m_speedMult = mult;
    }

    public void setRotationMult(double mult) {
        m_rotationMult = mult;
    }
}
