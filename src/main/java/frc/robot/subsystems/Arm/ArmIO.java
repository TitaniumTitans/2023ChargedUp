package frc.robot.subsystems.Arm;

import org.littletonrobotics.junction.AutoLog;


public interface ArmIO {

    boolean armAtLowerLimit();

    @AutoLog
    public static class ArmIOInputs {
        public double ArmAngle = 0.0;
        public double ArmExtensionLength = 0.0;
    }

    public default void updateInputs(ArmIOInputsAutoLogged inputs){}
    
    //Setters
    public default void setAngleSpeed(double speed) {}

    public default void setArmSpeed(double speed) {}

    public default void setArmExtension(double speed) {}
 

    //Getters
    public default double getArmAngle() {
        return 0.0;
    }

    public default double getArmExtension() {
        return 0.0;
    }

    public default void setArmAngle(double angle) {}

    public default boolean encoderConnected() {
        return false;
    }
    
    public default void resetExstentionEncoder() {}
}
