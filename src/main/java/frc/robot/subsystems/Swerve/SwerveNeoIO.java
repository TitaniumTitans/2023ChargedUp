package frc.robot.subsystems.Swerve;

import java.sql.Driver;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Swerve.SwerveModules.SwerveModNeo;

public class SwerveNeoIO implements SwerveIO{
    private SwerveModNeo m_frMod;
    private SwerveModNeo m_flMod;
    private SwerveModNeo m_blMod;
    private SwerveModNeo m_brMod;
    private SwerveModNeo[] m_modules;

    private PigeonIMU m_gyro;
    private SwerveDriveOdometry m_odometry;

    private boolean fieldRelative;

    public SwerveNeoIO(){
        m_flMod = new SwerveModNeo(0, DriveConstants.kMod0Offset, DriveConstants.kMod0Cans);
        m_frMod = new SwerveModNeo(1, DriveConstants.kMod1Offset, DriveConstants.kMod1Cans);
        m_blMod = new SwerveModNeo(2, DriveConstants.kMod2Offset, DriveConstants.kMod2Cans);
        m_brMod = new SwerveModNeo(3, DriveConstants.kMod3Offset, DriveConstants.kMod3Cans);
        m_modules = new SwerveModNeo[] {m_flMod, m_frMod, m_blMod, m_brMod};

        m_gyro = new PigeonIMU(DriveConstants.kGyroCan);

        m_odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getGyroYaw(), getModulePositions())

    }

    // Getters
    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] modPos = new SwerveModulePosition[4];

        for(int i = 0; i <= m_modules.length; i++){
            modPos[i] = m_modules[i].getPosition();
        }

    return modPos;
    }

    public Rotation2d getGyroYaw(){
        return Rotation2d.fromDegrees(m_gyro.getYaw());
    }

    // Setters
    public void setModuleStates(double xTranslation, double yTranslation, double zRotation){
        SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xTranslation, getGyroYaw()) 
        )
    }
    
}
