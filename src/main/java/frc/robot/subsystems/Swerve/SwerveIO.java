package frc.robot.subsystems.Swerve;

import org.littletonrobotics.junction.AutoLog;

public interface SwerveIO {

    @AutoLog
    public static class SwerveIOInputs{
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

    public default void updateInputs(SwerveIOInputs inputs){
    }

    public default void setModuleStates(double xTranslation, double yTranslation, double zRotation){
    }
}
