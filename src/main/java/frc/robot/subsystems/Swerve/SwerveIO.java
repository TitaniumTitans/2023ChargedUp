package frc.robot.subsystems.Swerve;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveIO {

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

    public default void updateInputs(SwerveIOInputs inputs) {}

    public default void setModuleStates(double xTranslation, double yTranslation, double zRotation, boolean fieldRelative) {}

    public default void setModuleStates(SwerveModuleState[] states) {}

    public default void periodic() {}

    public default void setAbsoluteAngles() {}

    public default void setIndevidualAngle(int module) {}

    public default SwerveModuleState[] getModuleStates() {
        return null;
    }

    public default SwerveModulePosition[] getModulePositions() {
        return null;
    }

    public default Rotation2d getGyroYaw() {
        return null;
    }

    public default Rotation2d getGyroRoll() {
        return null;
    }

    public default void resetGyro() {}

    public default Rotation2d[] getCancoder() {
        return null;
    }
}
