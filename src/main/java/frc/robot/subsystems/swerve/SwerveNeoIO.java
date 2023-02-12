package frc.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.swerve.SwerveModules.SwerveModNeo;

public class SwerveNeoIO implements SwerveIO {
    private SwerveModNeo m_frMod;
    private SwerveModNeo m_flMod;
    private SwerveModNeo m_blMod;
    private SwerveModNeo m_brMod;

    private PigeonIMU m_gyro;

    public SwerveNeoIO() {
        m_flMod = new SwerveModNeo(0, DriveConstants.MOD_FL_OFFSET, DriveConstants.MOD_FL_CANS, false);
        m_frMod = new SwerveModNeo(1, DriveConstants.MOD_FR_OFFSET, DriveConstants.MOD_FR_CANS, false);
        m_blMod = new SwerveModNeo(2, DriveConstants.MOD_BL_OFFSET, DriveConstants.MOD_BL_CANS, false);
        m_brMod = new SwerveModNeo(3, DriveConstants.MOD_BR_OFFSET, DriveConstants.MOD_BR_CANS, false);

        m_gyro = new PigeonIMU(DriveConstants.GYRO_CAN);
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
    @Override
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
        m_flMod.setDesiredState(states[1]);
        m_frMod.setDesiredState(states[0]);
        m_blMod.setDesiredState(states[3]);
        m_brMod.setDesiredState(states[2]);
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
    public void setAbsoluteAngles() {
        m_flMod.resetToAbsolute();
        m_frMod.resetToAbsolute();
        m_blMod.resetToAbsolute();
        m_brMod.resetToAbsolute();
    }

    @Override
    public void setIndevidualAngle(int module) {
        switch (module) {
            case 1:
                m_flMod.resetToAbsolute();
                break;
            case 2:
                m_frMod.resetToAbsolute();
                break;
            case 3:
                m_blMod.resetToAbsolute();
                break;
            case 4: 
                m_brMod.resetToAbsolute();
                break;
            default:
                setAbsoluteAngles();
        }
    }

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

    @Override
    public double[] getAngles(){
        return new double[]{
                m_flMod.getAngle(),
                m_frMod.getAngle(),
                m_blMod.getAngle(),
                m_brMod.getAngle()
        };
    }

    @Override
    public Rotation2d[] getCancoderAngles() {
        return new Rotation2d[] {
                m_flMod.getCanCoder(),
                m_frMod.getCanCoder(),
                m_blMod.getCanCoder(),
                m_brMod.getCanCoder(),
        };
    }
    
}
