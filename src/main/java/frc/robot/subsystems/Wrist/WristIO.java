package frc.robot.subsystems.Wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {

    @AutoLog    
    public static class WristIOInputs {
        public double wristAngle;
        public double intakeAmps;
    }

    public default void updateInputs(WristIOInputsAutoLogged inputs){}
    
    //Setters
    public default void setWristAngle(double angle) {}

    //Getters
    public default double getWristAngle() {
        return 0.0;
    }

    public default double getIntakeAmps() {
        return 0.0;
    }
}
