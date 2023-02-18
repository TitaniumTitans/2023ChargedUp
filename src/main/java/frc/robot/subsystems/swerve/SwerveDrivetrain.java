package frc.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.swerve.module.SwerveModNeo;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class SwerveDrivetrain extends SubsystemBase {
    private final SwerveModNeo m_frMod;
    private final SwerveModNeo m_flMod;
    private final SwerveModNeo m_blMod;
    private final SwerveModNeo m_brMod;

    private final PigeonIMU m_gyro;

    private final SwerveIOInputsAutoLogged m_inputs;

    private boolean fieldOriented = false;

    private final Field2d m_field;
    private final SwerveDrivePoseEstimator m_poseEstimator;

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

        m_gyro = new PigeonIMU(DriveConstants.GYRO_CAN);
        m_inputs = new SwerveIOInputsAutoLogged();

        m_field = new Field2d();
        SmartDashboard.putData("Field", m_field);
        m_poseEstimator = new SwerveDrivePoseEstimator(
                DriveConstants.DRIVE_KINEMATICS,
                getGyroYaw(),
                getModulePositions(),
                new Pose2d()
        );
    }

    @Override
    public void periodic() {
        updateInputs();
        Logger.getInstance().processInputs("Swerve", m_inputs);

        updatePoseEstimator();
        m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());
    }

    // Getters
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] modPos = new SwerveModulePosition[4];

        modPos[1] = m_flMod.getPosition();
        modPos[0] = m_frMod.getPosition();
        modPos[3] = m_blMod.getPosition();
        modPos[2] = m_brMod.getPosition();

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
        return Rotation2d.fromDegrees((m_gyro.getYaw() * -1));
    }


    // Setters
    public void setModuleStates(double xTranslation, double yTranslation, double zRotation) {
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
        m_flMod.setDesiredState(states[1]);
        m_frMod.setDesiredState(states[0]);
        m_blMod.setDesiredState(states[3]);
        m_brMod.setDesiredState(states[2]);
    }

    public void resetGyro() {
        m_gyro.setYaw(0);
    }

    public Rotation2d getGyroRoll() {
        return Rotation2d.fromDegrees(m_gyro.getRoll());
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
        m_poseEstimator.update(getGyroYaw(), getModulePositions());
    }

    public void resetPose(Pose2d newPose) {
        m_poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), newPose);
    }

    public Pose2d getPose() {
        double estPoseX = m_poseEstimator.getEstimatedPosition().getX();
        double estPoseY = m_poseEstimator.getEstimatedPosition().getY();

        return new Pose2d(estPoseX, estPoseY * -1, new Rotation2d());
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

    public Command resetGyroBase() {
        return runOnce(this::resetGyro);
    }

    public Command toggleFieldRelative() {
        return runOnce(() -> fieldOriented = !fieldOriented);
    }

    public CommandBase resetPoseBase() {
        return runOnce(() -> resetPose(new Pose2d()));
    }
}
