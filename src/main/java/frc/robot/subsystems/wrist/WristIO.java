package frc.robot.subsystems.wrist;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
    @AutoLog
    class WristIOInputs {
        public boolean wristAtSetpoint = false;
        public double wristAngle = 0.0;
        public boolean pieceInsideIntake = false;
        public boolean wristAtLowerLimit = false;
        public boolean wristAtUpperLimit = false;

        public double wristOutputRaw = 0.0;
        public double wristAmpOutput = 0.0;
        public double wristTemperature = 0.0;
        public double wristPidOutput = 0.0;
        public double wristPidSetpoint = 0.0;

        public double intakeTemperature = 0.0;
        public double intakeAmpOutput = 0.0;

        public boolean wristHomed = false;
        public boolean wristStalling = false;
        public boolean intakeStalling = false;
    }

    default void update(WristIOInputsAutoLogged inputs) {}

    default void setIntakePower(double speed) {}

    default void setWristAngle(double angle, double setpoint) {}

    default void setWristPower(double speed) {}

    void setBrakeMode(CANSparkMax.IdleMode brakeMode);
    default void toggleBrakeMode() {};

    default void resetHomed() {}

    default void goToHome() {}

    default void zeroWrist() {}
}
