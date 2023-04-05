package frc.robot.subsystems.swerve.module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleInterface {
    default void setDesiredState(SwerveModuleState state) {}

    default SwerveModuleState getModuleState() { return new SwerveModuleState();}

    default SwerveModulePosition getModulePosition() {return new SwerveModulePosition();}

    default Rotation2d getAbsoluteAngle() {return  new Rotation2d();}

    default Rotation2d getModuleAngle() {return new Rotation2d();}

    default void setMagnetOffset() {}
}
