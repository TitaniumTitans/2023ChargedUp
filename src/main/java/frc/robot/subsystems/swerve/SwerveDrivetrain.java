package frc.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
import frc.robot.subsystems.vision.CameraSubsystem;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import java.util.HashMap;
import java.util.Optional;

public class SwerveDrivetrain extends SubsystemBase {
    private final SwerveModNeo m_frMod;
    private final SwerveModNeo m_flMod;
    private final SwerveModNeo m_blMod;
    private final SwerveModNeo m_brMod;

    private final WPI_Pigeon2 m_gyro;

    private final SwerveIOInputsAutoLogged m_inputs;

    private boolean fieldOriented = false;

    private final Field2d m_field;
    private final SwerveDrivePoseEstimator m_poseEstimator;
    private final CameraSubsystem m_frontCamSubsystem;
//    private final CameraSubsystem m_leftCamSubsystem;


    private boolean slowmode = false;
    private double m_currentRoll = 0;
    private double m_previousRoll = 0;

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

        m_field = new Field2d();
        SmartDashboard.putData("Field", m_field);
        m_poseEstimator = new SwerveDrivePoseEstimator(
                DriveConstants.DRIVE_KINEMATICS,
                getGyroYaw(),
                getModulePositions(),
                new Pose2d(),
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.1),
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.5, 0.5, 0.5)
        );

        m_frontCamSubsystem = new CameraSubsystem(DriveConstants.FRONT_CAM_NAME, DriveConstants.FRONT_CAM_POSE);
//        m_leftCamSubsystem = new CameraSubsystem(DriveConstants.LEFT_CAM_NAME, DriveConstants.LEFT_CAM_POSE);

        SmartDashboard.putData("Field", m_field);

    }

    @Override
    public void periodic() {
        updateInputs();
        Logger.getInstance().processInputs("Swerve", m_inputs);

        updatePoseEstimator();
        m_field.setRobotPose(getPose());

        double[] angles = getAngles();
        SmartDashboard.putNumber("Swerve Gyro Yaw", getGyroYaw().getDegrees());

        SmartDashboard.putNumber("FL Angle", angles[0]);
        SmartDashboard.putNumber("FR Angle", angles[1]);
        SmartDashboard.putNumber("BL Angle", angles[2]);
        SmartDashboard.putNumber("BR Angle", angles[3]);

        Rotation2d[] cancoderAngles = getCancoderAngles();
        SmartDashboard.putNumber("FL Cancoder", cancoderAngles[0].getDegrees());
        SmartDashboard.putNumber("FR Cancoder", cancoderAngles[1].getDegrees());
        SmartDashboard.putNumber("BL Cancoder", cancoderAngles[2].getDegrees());
        SmartDashboard.putNumber("BR Cancoder", cancoderAngles[3].getDegrees());

        m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());

        m_previousRoll = m_currentRoll;
        m_currentRoll = getGyroPitch().getDegrees();
    }

    // Getters
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] modPos = new SwerveModulePosition[4];

        modPos[0] = m_flMod.getPosition();
        modPos[1] = m_frMod.getPosition();
        modPos[2] = m_blMod.getPosition();
        modPos[3] = m_brMod.getPosition();

    return modPos;
    }


    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];

        states[0] = m_flMod.getState();
        states[1] = m_frMod.getState();
        states[2] = m_blMod.getState();
        states[3] = m_brMod.getState();

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
                zRotation,
                getGyroYaw()) :
            new ChassisSpeeds(
                xTranslation,
                yTranslation,
                zRotation
            ));

    setModuleStates(states);
    }

    public void setModuleStates(SwerveModuleState[] states) {
        m_flMod.setDesiredState(states[0]);
        m_frMod.setDesiredState(states[1]);
        m_blMod.setDesiredState(states[2]);
        m_brMod.setDesiredState(states[3]);

        SmartDashboard.putNumber("Desired angle FL", states[0].angle.getDegrees());
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

    public double getGyroRollPitch() {
        return m_currentRoll - m_previousRoll;
    }

    public void setAbsoluteAngles() {
        m_flMod.resetToAbsolute();
        m_frMod.resetToAbsolute();
        m_blMod.resetToAbsolute();
        m_brMod.resetToAbsolute();
    }

    public void updateInputs() {
        m_inputs.flAngleDeg = m_flMod.getState().angle.getDegrees();
        m_inputs.flDriveSpeedMPS = m_flMod.getState().speedMetersPerSecond;

        m_inputs.frAngleDeg = m_frMod.getState().angle.getDegrees();
        m_inputs.frDriveSpeedMPS = m_frMod.getState().speedMetersPerSecond;

        m_inputs.blAngleDeg = m_blMod.getState().angle.getDegrees();
        m_inputs.blDriveSpeedMPS = m_blMod.getState().speedMetersPerSecond;

        m_inputs.brAngleDeg = m_brMod.getState().angle.getDegrees();
        m_inputs.brDriveSpeedMPS = m_brMod.getState().speedMetersPerSecond;

        m_inputs.gyroPitchDeg = m_gyro.getPitch();
        m_inputs.gyroYawDeg = m_gyro.getYaw();
    }

    public void updatePoseEstimator() {
        /**
         * Get swerve odometry
         */
        m_poseEstimator.update(getGyroYaw(), getModulePositions());

        /**
         * Create new vision poses for each cam
         */
        Optional<EstimatedRobotPose> frontCamEstimatePose =
                m_frontCamSubsystem.getPose(getPose());
//        Optional<EstimatedRobotPose> leftCamEstimatePose =
//                m_leftCamSubsystem.getPose(getPose());

        SmartDashboard.putBoolean("FC pose present", frontCamEstimatePose.isPresent());
//        SmartDashboard.putBoolean("LC pose present", leftCamEstimatePose.isPresent());

        /**
         * Add each vision measurement to the pose estimator if it exists for each camera
         */
        if(frontCamEstimatePose.isPresent()) {
            EstimatedRobotPose frontCamPose = frontCamEstimatePose.get();

            SmartDashboard.putNumber("FC pose X", frontCamPose.estimatedPose.getX());
            SmartDashboard.putNumber("FC pose Y", frontCamPose.estimatedPose.getY());

            m_poseEstimator.addVisionMeasurement(frontCamPose.estimatedPose.toPose2d(), frontCamPose.timestampSeconds);
        }
//        if(leftCamEstimatePose.isPresent()) {
//            EstimatedRobotPose leftCamPose = leftCamEstimatePose.get();
//
//            SmartDashboard.putNumber("LC pose X", leftCamPose.estimatedPose.getX());
//            SmartDashboard.putNumber("LC pose Y", leftCamPose.estimatedPose.getY());
//
//            m_poseEstimator.addVisionMeasurement(leftCamPose.estimatedPose.toPose2d(), leftCamPose.timestampSeconds);
//        }
    }

    public Pose2d getFrontCamTagPose() {
        return m_frontCamSubsystem.getTagPose();
    }

    public void resetPose(Pose2d newPose) {
        m_poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), newPose);
    }

    public Pose2d getPose() {
//        double estPoseX = m_poseEstimator.getEstimatedPosition().getX();
//        double estPoseY = m_poseEstimator.getEstimatedPosition().getY();

//        return new Pose2d(estPoseX, estPoseY * -1, new Rotation2d());
        return m_poseEstimator.getEstimatedPosition();
    }

    public double[] getAngles(){
        return new double[]{
                m_flMod.getAngle(),
                m_frMod.getAngle(),
                m_blMod.getAngle(),
                m_brMod.getAngle()
        };
    }

    public Rotation2d[] getCancoderAngles() {
        return new Rotation2d[] {
                m_flMod.getCanCoder(),
                m_frMod.getCanCoder(),
                m_blMod.getCanCoder(),
                m_brMod.getCanCoder(),
        };
    }

    @SuppressWarnings("PMD.AvoidReassigningParameters")
    public Command followPPTrajectory(AlignmentOptions align) {
        // Figure out what pose the robot should be
        Pose2d tagPose = getFrontCamTagPose();

        Translation2d translatedEnd;
        Translation2d translatedMiddle;
        PathPlannerTrajectory traj;

        switch(align){
            case HUMAN_PLAYER_ALIGN:
            case LEFT_ALIGN:
                translatedEnd = tagPose.transformBy(AutoConstants.LEFT_TRANSLATION).getTranslation();
                break;
            case RIGHT_ALIGN:
                translatedEnd = tagPose.transformBy(AutoConstants.RIGHT_TRANSLATION).getTranslation();
                break;
            default:
                translatedEnd = tagPose.transformBy(AutoConstants.CENTER_TRANSLATION).getTranslation();
        }

        translatedMiddle = tagPose.getRotation().getDegrees() == 180 ?
                translatedEnd.minus(new Translation2d(0.25, 0)) :
                translatedEnd.plus(new Translation2d(0.25, 0));


        //generate a path based on the tag you see, flipped 180 from tag pose
        traj = PathPlanner.generatePath(
                AutoUtils.getDefaultConstraints(),
                new PathPoint(getPose().getTranslation(), new Rotation2d(), getPose().getRotation()),
                new PathPoint(translatedMiddle, new Rotation2d(), tagPose.getRotation().rotateBy(Rotation2d.fromDegrees(180))),
                new PathPoint(translatedEnd, new Rotation2d(), tagPose.getRotation().rotateBy(Rotation2d.fromDegrees(180)))
        );

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

    public SwerveAutoBuilder getAutoBuilder(HashMap<String, Command> eventMap) {
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

    public Command toggleFieldRelative() {
        return runOnce(() -> fieldOriented = !fieldOriented);
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
        return runOnce(() -> {
            setSlowmode();
        });
    }

    public CommandBase createPPSwerveController(AlignmentOptions align) {
        return new ProxyCommand(() -> followPPTrajectory(align));
    }
}
