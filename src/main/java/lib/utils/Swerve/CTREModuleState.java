package lib.utils.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class CTREModuleState {

    /**
     * Minimizes the rotation done by the azimuth motor by potentially reversing 
     * the direction the wheel spins. Customized from WPILib's version, but allowing for
     * CTRE's closed loop position control
     * 
     * @param desiredState The desired state
     * @param currentAngle the current module angle
     * @return 
     */
    public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle){
        double targetSpeed = desiredState.speedMetersPerSecond;
        double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());

        double delta = targetAngle - currentAngle.getDegrees();
        if (Math.abs(delta) > 90){
            targetSpeed = -targetSpeed;
            targetAngle = delta > 90 ? (targetAngle - 180) : (targetAngle + 180);
        }
        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    }

    /***
     * Wraps the current position of the swerve module around a
     * 0 - 360 boundary so that it can be used in the optimize method
     * 
     * @param scopeReference The current angle the swerve module is at
     * @param newAngle
     * @return
     */
    public static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;

        if(lowerOffset >= 0 ){
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        }else {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }
        while (newAngle < lowerBound) {
            newAngle += 360;
        }
        while (newAngle > upperBound) {
            newAngle -= 360;
        }
        if (newAngle - scopeReference > 180) {
            newAngle -= 360;
        } else if (newAngle - scopeReference < -180) {
            newAngle += 360;
        }
        return newAngle;
    }
}
