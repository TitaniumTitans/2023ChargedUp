package frc.robot.subsystems.swerve.module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {
    @AutoLog
    class SwerveModuleInputs {
        public double azimuthAngle = 0.0;
        public double azimuthAbsoluteAngle = 0.0;
        public double azimuthCurrentDraw = 0.0;
        public double azimuthVelocityDegPerSec = 0.0;
        public double azimuthAppliedVolts = 0.0;

        public double drivePosition = 0.0;
        public double driveVelocityMPS = 0.0;
        public double driveSpeedDegPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
    }

    default void updateInputs(SwerveModuleInputsAutoLogged inputs) {}

    default SwerveModuleState setDesiredState(SwerveModuleState state) {return new SwerveModuleState();}

    default SwerveModuleState getModuleState() { return new SwerveModuleState();}

    default SwerveModulePosition getModulePosition() {return new SwerveModulePosition();}

    default Rotation2d getAbsoluteAngle() {return  new Rotation2d();}

    default Rotation2d getModuleAngle() {return new Rotation2d();}

    default void setMagnetOffset() {}
}
