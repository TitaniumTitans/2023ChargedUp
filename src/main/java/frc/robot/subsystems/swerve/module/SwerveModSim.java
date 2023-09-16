package frc.robot.subsystems.swerve.module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.swerve.SwerveModuleSim;
import frc.robot.Constants;

public class SwerveModSim implements SwerveModuleIO {
    SwerveModuleSim m_moduleSim;
    
    public SwerveModSim() {
        m_moduleSim = new SwerveModuleSim(
                DCMotor.getFalcon500(1),
                DCMotor.getFalcon500(1),
                Constants.ModuleConstants.WHEEL_DIAMETER_METERS,
                Constants.ModuleConstants.TURNING_RATIO,
                Constants.ModuleConstants.L3_GEAR_RATIO
        );
    }


    @Override
    public void updateInputs(SwerveModuleInputsAutoLogged inputs) {
        inputs.azimuthAbsoluteAngleDegrees = Math.toDegrees(m_moduleSim.getAzimuthEncoderPositionRads());
        inputs.azimuthAngleDegrees = inputs.azimuthAbsoluteAngleDegrees;

        inputs.driveVelocityMPS = m_moduleSim.getWheelEncoderMetersPerSecond();

        m_moduleSim.update(0.02);
    }

    @Override
    public void setDesiredState(SwerveModuleState state) {
        double azimuthVoltage = ((state.angle.getRadians() - m_moduleSim.getAzimuthEncoderPositionRads()) * 0.5) % 12;
        double driveVoltage = state.speedMetersPerSecond;
        m_moduleSim.setInputVoltages(driveVoltage, azimuthVoltage);
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
                m_moduleSim.getWheelEncoderPositionRev() * Constants.ModuleConstants.WHEEL_DIAMETER_METERS,
                new Rotation2d(m_moduleSim.getAzimuthEncoderPositionRads())
        );
    }

    @Override
    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(
                m_moduleSim.getWheelEncoderMetersPerSecond(),
                new Rotation2d(m_moduleSim.getAzimuthEncoderPositionRads())
        );
    }
}
