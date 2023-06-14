package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
    @AutoLog
    class WristIOInputs {
        public boolean wristAtSetpoint = false;
        public boolean pieceInsideIntake = false;
        public boolean wristAtLowerLimit = false;
        public boolean wristAtUpperLimit = false;

        public double wristOutputRaw = 0.0;
        public double wristAmpOutput = 0.0;
        public double wristTemperature = 0.0;
        public double wristPidOutput = 0.0;

        public double intakeTemperature = 0.0;
        public double intakeAmpOutput = 0.0;

        public boolean wristHomed = false;
        public boolean wristStalling = false;
        public boolean intakeStalling = false;
    }

    default void updateInputs(WristIOInputsAutoLogged inputs) {}

    default void setIntakePower(double speed) {}

    default void setIntakePowerFactory(double speed) {}

    default void setWristAngle(double angle) {}

    default void setWristPower(double speed) {}

    default void setWristAngleFactory(double angle) {}

    default void setWristPowerFactory(double speed) {}

    default void setBrakeMode(boolean brakeMode) {}

    default void resetHomed() {}

    default void goToHome() {}

    default void zeroWrist() {}
}
