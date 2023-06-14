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
import frc.robot.subsystems.swerve.module.FalconProModule;
import frc.robot.subsystems.swerve.module.SwerveModNeo;
import frc.robot.subsystems.swerve.module.SwerveModuleIO;
import frc.robot.subsystems.vision.CameraSubsystem;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

public class SwerveDrivetrain extends SubsystemBase {
    private final SwerveModuleIO m_frMod;
    private final SwerveModuleIO m_flMod;
    private final SwerveModuleIO m_blMod;
    private final SwerveModuleIO m_brMod;

    private final WPI_Pigeon2 m_gyro;
    private final TimeOfFlight m_tofSensor;

    private boolean fieldOriented = true;

    private final Field2d m_field;
    private final SwerveDrivePoseEstimator m_poseEstimator;
    private final SwerveDrivePoseEstimator m_visionEstimator;

    private final CameraSubsystem m_leftCam;
    private final CameraSubsystem m_rightCam;


    private double m_currentPitch = 0;
    private double m_previousPitch = 0;
    private double m_currentTime = 0;
    private double m_prevTime = 0;
    private boolean slowmode = false;
    private double m_speedMult = 1;
    private double m_rotationMult = 1;

    public enum AlignmentOptions {
        LEFT_ALIGN,
        CENTER_ALIGN,
        RIGHT_ALIGN,
        HUMAN_PLAYER_ALIGN
    }

    public SwerveDrivetrain() {
        // Check current robot mode for the proper hardware
        if (Constants.CURRENT_MODE == Constants.Mode.HELIOS_V1) {
            m_flMod = new SwerveModNeo(0, DriveConstants.MOD_FL_OFFSET, DriveConstants.MOD_FL_CANS, false);
            m_frMod = new SwerveModNeo(1, DriveConstants.MOD_FR_OFFSET, DriveConstants.MOD_FR_CANS, false);
            m_blMod = new SwerveModNeo(2, DriveConstants.MOD_BL_OFFSET, DriveConstants.MOD_BL_CANS, false);
            m_brMod = new SwerveModNeo(3, DriveConstants.MOD_BR_OFFSET, DriveConstants.MOD_BR_CANS, false);
        } else {
            m_flMod = new FalconProModule(DriveConstants.MOD_FL_OFFSET_V2, DriveConstants.MOD_FL_CANS);
            m_frMod = new FalconProModule(DriveConstants.MOD_FR_OFFSET_V2, DriveConstants.MOD_FR_CANS);
            m_blMod = new FalconProModule(DriveConstants.MOD_BL_OFFSET_V2, DriveConstants.MOD_BL_CANS);
            m_brMod = new FalconProModule(DriveConstants.MOD_BR_OFFSET_V2, DriveConstants.MOD_BR_CANS);
        }

        // open gyro and ToF sensor
        m_gyro = new WPI_Pigeon2(DriveConstants.GYRO_CAN);

        m_tofSensor = new TimeOfFlight(Constants.WristConstants.TOF_PORT);
        m_tofSensor.setRangingMode(TimeOfFlight.RangingMode.Short, 10);

        // open field data and advantagekit outputs
        m_field = new Field2d();

        // construct the pose estimator for odometry
        m_poseEstimator = new SwerveDrivePoseEstimator(
                DriveConstants.DRIVE_KINEMATICS,
                getGyroYaw(),
                getModulePositions(),
                new Pose2d(),
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.1),
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.75, 0.75, Units.degreesToRadians(1.5))
        );

        // construct a secondary estimator for testing with cameras
        m_visionEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.DRIVE_KINEMATICS, 
            new Rotation2d(), 
            getModulePositions(), 
            new Pose2d());

        // Construct cameras
        m_leftCam = new CameraSubsystem(DriveConstants.LEFT_GLOBAL_CAM, DriveConstants.LEFT_CAM_POSE);
        m_rightCam = new CameraSubsystem(DriveConstants.RIGHT_GLOBAL_CAM, DriveConstants.RIGHT_CAM_POSE);


        // Setup field and initialize gyro
        SmartDashboard.putData("Field", m_field);
        resetGyro();
        m_visionEstimator.update(new Rotation2d(), getModulePositions());
    }

    @Override
    public void periodic() {
        // Update logged values
        Logger.getInstance().recordOutput("Odometry/Robot Pose", getPose());

        updatePoseEstimator();
        m_field.setRobotPose(getPose());

        SwerveModuleState[] states = getModuleStates();
        for (SwerveModuleState s : states) {

        }

        m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());

        // Record prev and current pitch and time used for auto balance
        m_previousPitch = m_currentPitch;
        m_currentPitch = getGyroPitch().getDegrees();

        m_prevTime = m_currentTime;
        m_currentTime = RobotController.getFPGATime();
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
//            if (SmartDashboard.getBoolean("Stella Mode", true)) {
//                state.speedMetersPerSecond *= 0.2;
//            }
       }

        m_flMod.setDesiredState(states[0]);
        m_frMod.setDesiredState(states[1]);
        m_blMod.setDesiredState(states[2]);
        m_brMod.setDesiredState(states[3]);
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
        //Account for initail boot time
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

    public void updatePoseEstimator() {
        /*
         * Get swerve odometry
         */
        m_poseEstimator.update(getGyroYaw(), getModulePositions());

        /*
         * Create new vision poses for each cam
         */

        Optional<EstimatedRobotPose> leftCamPose = m_leftCam.getPose(getPose());
        Optional<EstimatedRobotPose> rightCamPose = m_rightCam.getPose(getPose());

        Logger.getInstance().recordOutput("L Cam Pose", leftCamPose.isPresent() ? leftCamPose.get().estimatedPose.toPose2d() : new Pose2d());
        Logger.getInstance().recordOutput("R Cam Pose", rightCamPose.isPresent() ? rightCamPose.get().estimatedPose.toPose2d() : new Pose2d());


        /*
         * Add each vision measurement to the pose estimator if it exists for each camera
         */

        leftCamPose.ifPresent(estimatedRobotPose -> m_visionEstimator.addVisionMeasurement(estimatedRobotPose.estimatedPose.toPose2d(), leftCamPose.get().timestampSeconds));
        rightCamPose.ifPresent(estimatedRobotPose -> m_visionEstimator.addVisionMeasurement(estimatedRobotPose.estimatedPose.toPose2d(), rightCamPose.get().timestampSeconds));

        Logger.getInstance().recordOutput("Merged Cam pose", m_visionEstimator.getEstimatedPosition());

        Logger.getInstance().recordOutput("Left Cam Has Pose", leftCamPose.isPresent());
        Logger.getInstance().recordOutput("Right Cam Has Pose", rightCamPose.isPresent());

//         leftCamPose.ifPresent(estimatedRobotPose -> m_poseEstimator.addVisionMeasurement(estimatedRobotPose.estimatedPose.toPose2d(), leftCamPose.get().timestampSeconds));
//         rightCamPose.ifPresent(estimatedRobotPose -> m_poseEstimator.addVisionMeasurement(estimatedRobotPose.estimatedPose.toPose2d(), rightCamPose.get().timestampSeconds));
    }

    public Pose2d getFrontCamTagPose() {
        return new Pose2d();
    }

    public int getFrontCamTagID() {
        return 0;
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

    public Command alignToTag(AlignmentOptions align) {
        // Figure out what pose the robot should be
        Pose2d tagPose = getFrontCamTagPose();
        int tagID = getFrontCamTagID();

        Translation2d translatedEnd;
        Translation2d translatedMiddle;
        PathPlannerTrajectory traj;

        switch (align) {
            case HUMAN_PLAYER_ALIGN:
                translatedEnd = tagPose.transformBy(AutoConstants.HUMAN_PLAYER_RIGHT_TRANSLATION).getTranslation();
                break;
            case LEFT_ALIGN:
                if (tagID == 4 || tagID == 5) {
                    translatedEnd = tagPose.transformBy(AutoConstants.HUMAN_PLAYER_LEFT_TRANSLATION).getTranslation();
                } else {
                    translatedEnd = tagPose.transformBy(AutoConstants.LEFT_TRANSLATION).getTranslation();
                }
                break;
            case RIGHT_ALIGN:
                if (tagID == 4 || tagID == 5) {
                    translatedEnd = tagPose.transformBy(AutoConstants.HUMAN_PLAYER_RIGHT_TRANSLATION).getTranslation();
                } else {
                    translatedEnd = tagPose.transformBy(AutoConstants.RIGHT_TRANSLATION).getTranslation();
                }
                break;
            default:
                translatedEnd = tagPose.transformBy(AutoConstants.CENTER_TRANSLATION).getTranslation();
        }

        //translate to meters but agressive
        //offset theh alignment by the pieces position in the intake
        double tofOffset = (getDetectionRange() - 100) * 0.02;
        translatedEnd.minus(new Translation2d(tofOffset, 0.0));

        translatedMiddle = tagPose.getRotation().getDegrees() == 180 ?
                translatedEnd.minus(new Translation2d(0.25, 0)) :
                translatedEnd.plus(new Translation2d(0.25, 0));

        Translation2d chassisSpeed = new Translation2d(
                getChassisSpeed().vxMetersPerSecond,
                getChassisSpeed().vyMetersPerSecond
        );


        //generate a path based on the tag you see, flipped 180 from tag pose
        traj = PathPlanner.generatePath(
                AutoUtils.getDefaultConstraints(),
                new PathPoint(getPose().getTranslation(), new Rotation2d(), getPose().getRotation(), chassisSpeed.getNorm()),
                new PathPoint(translatedMiddle, new Rotation2d(), tagPose.getRotation().rotateBy(Rotation2d.fromDegrees(180))),
                new PathPoint(translatedEnd, new Rotation2d(), tagPose.getRotation().rotateBy(Rotation2d.fromDegrees(180)))
        );

        Logger.getInstance().recordOutput("Current Trajectory", traj);

        return new PPSwerveControllerCommand(
                traj,
                this::getPose,
                DriveConstants.DRIVE_KINEMATICS,
                Constants.AutoConstants.CONTROLLER_X,
                Constants.AutoConstants.CONTROLLER_Y,
                Constants.AutoConstants.THETA_CONTROLLER,
                this::setModuleStates,
                this
        );
    }

    public void followTag(Translation2d offset) {
        // Determine the "end point" for the follow
        Transform3d tagToRobot = m_leftCam.robotToTag();
        tagToRobot.plus(new Transform3d(new Translation3d(offset.getX(), offset.getY(), 0.0), new Rotation3d()));

        double xOut = MathUtil.clamp(DriveConstants.FOLLOW_CONTROLLER_X.calculate(getPose().getX(), tagToRobot.getX()), -1.0, 1.0);
        double yOut = MathUtil.clamp(DriveConstants.FOLLOW_CONTROLLER_Y.calculate(getPose().getY(), tagToRobot.getY()), -1.0, 1.0);
        double zOut = MathUtil.clamp(DriveConstants.FOLLOW_CONTROLLER_THETA.calculate(getPose().getRotation().getDegrees(), tagToRobot.getRotation().toRotation2d().getDegrees()), -1.0, 1.0);

        drive(xOut, yOut, zOut);
    }

    public ChassisSpeeds getChassisSpeed() {
        return DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    public double getDetectionRange() {
        return m_tofSensor.getRange();
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

    // Command factories and their respective methods
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

    public CommandBase createPPSwerveController(AlignmentOptions align) {
        return new ProxyCommand(() -> alignToTag(align));
    }

    public void setSpeedMult(double mult) {
        m_speedMult = mult;
    }

    public void setRotationMult(double mult) {
        m_rotationMult = mult;
    }
}
