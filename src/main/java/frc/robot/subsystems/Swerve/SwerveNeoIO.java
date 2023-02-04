package frc.robot.subsystems.Swerve;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Swerve.SwerveModules.SwerveModNeo;

public class SwerveNeoIO implements SwerveIO {
    private SwerveModNeo m_frMod;
    private SwerveModNeo m_flMod;
    private SwerveModNeo m_blMod;
    private SwerveModNeo m_brMod;
    // private SwerveModNeo[] m_modules;

    private PigeonIMU m_gyro;
    // private SwerveDriveOdometry m_odometry;

    // private Field2d m_field;

    public SwerveNeoIO() {
        m_flMod = new SwerveModNeo(0, DriveConstants.MOD_FL_OFFSET, DriveConstants.MOD_FL_CANS, false);
        m_frMod = new SwerveModNeo(1, DriveConstants.MOD_FR_OFFSET, DriveConstants.MOD_FR_CANS, false);
        m_blMod = new SwerveModNeo(2, DriveConstants.MOD_BL_OFFSET, DriveConstants.MOD_BL_CANS, false);
        m_brMod = new SwerveModNeo(3, DriveConstants.MOD_BR_OFFSET, DriveConstants.MOD_BR_CANS, false);
        // m_modules = new SwerveModNeo[] {m_flMod, m_frMod, m_blMod, m_brMod};

        m_gyro = new PigeonIMU(DriveConstants.GYRO_CAN);

        // m_odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getGyroYaw(), getModulePositions());
        // m_field = new Field2d();
        // SmartDashboard.putData("Field", m_field);
    }

    // Getters
    @Override
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] modPos = new SwerveModulePosition[4];

        modPos[0] = m_flMod.getPosition();
        modPos[1] = m_frMod.getPosition();
        modPos[2] = m_blMod.getPosition();
        modPos[3] = m_brMod.getPosition();

    return modPos;
    }

    @Override
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];

        states[0] = m_flMod.getState();
        states[1] = m_frMod.getState();
        states[2] = m_blMod.getState();
        states[3] = m_brMod.getState();

        return states;
    }

    @Override
    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(m_gyro.getYaw());
    }

    // Setters
    public void setModuleStates(double xTranslation, double yTranslation, double zRotation, boolean fieldRelative) {
        SwerveModuleState[] states = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
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

    @Override
    public void setModuleStates(SwerveModuleState[] states) {
        m_flMod.setDesiredState(states[0]);
        m_frMod.setDesiredState(states[1]);
        m_blMod.setDesiredState(states[2]);
        m_brMod.setDesiredState(states[3]);
    }

    @Override
    public void resetGyro() {
        m_gyro.setYaw(0);
    }

    @Override
    public Rotation2d getGyroRoll() {
        return Rotation2d.fromDegrees(m_gyro.getRoll());
    }

    @Override
    public void setAbsoluteAngles() {}

    @Override
    public void updateInputs(SwerveIOInputs inputs) {
        inputs.flAngleDeg = m_flMod.getState().angle.getDegrees();
        inputs.flDriveSpeedMPS = m_flMod.getState().speedMetersPerSecond;

        inputs.frAngleDeg = m_frMod.getState().angle.getDegrees();
        inputs.frDriveSpeedMPS = m_frMod.getState().speedMetersPerSecond;

        inputs.blAngleDeg = m_blMod.getState().angle.getDegrees();
        inputs.blDriveSpeedMPS = m_blMod.getState().speedMetersPerSecond;

        inputs.brAngleDeg = m_brMod.getState().angle.getDegrees();
        inputs.brDriveSpeedMPS = m_brMod.getState().speedMetersPerSecond;

        inputs.gyroPitchDeg = m_gyro.getPitch();
        inputs.gyroYawDeg = m_gyro.getYaw();
    }

    public Rotation2d[] getCancoder(){
        Rotation2d[] array = {
            m_flMod.getCanCoder(),
            m_frMod.getCanCoder(),
            m_blMod.getCanCoder(),
            m_brMod.getCanCoder()
        };

        return array;
    }

    @Override
    public void periodic() {
        // m_odometry.update(Rotation2d.fromDegrees(m_gyro.getYaw()), getModulePositions());
        // m_field.setRobotPose(m_odometry.getPoseMeters());
        // SmartDashboard.putData("Field", m_field);
    }
    
}
