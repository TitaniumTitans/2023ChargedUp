package lib.utils;

/**
 * A collection of static utility methods.
 */
public class Utils {
    /**
     * Joystick deadband - The absolute value of the joystick must be larger than a
     * constant for an output to be returned.
     * 
     * @param input The current joystick input value
     * @return The output
     */
    public static double deadBand(double input) {
        if (Math.abs(input) < 0.2) {
            return 0.0;
        }
        return input;
    }

    /***
     * Converts RPM to Falcon Encoder Counts per 100 ms.
     * 
     * @param RPM Input RPM 
     * @param gearRatio Mechanism RPM
     * @return Falcon Velocity
     */
    public static double RPMToFalcon(double RPM, double gearRatio){
        double motorRPM = RPM * gearRatio;
        double sensorCounts = motorRPM * (2048.0 / 600.0);
        return sensorCounts;
    }

    /**
     * @param velocityCounts Falcon Velocity Counts
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return RPM of Mechanism
     */
    public static double falconToRPM(double velocityCounts, double gearRatio) {
        double motorRPM = velocityCounts * (600.0 / 2048.0);        
        double mechRPM = motorRPM / gearRatio;
        return mechRPM;
    }

    public static double MPSToFalcon(double velocity, double circumference, double gearRatio){
        double wheelRPM = ((velocity * 60) /circumference);
        double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
        return wheelVelocity;
    }

    /**
     * @param velocitycounts Falcon Velocity Counts
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return Falcon Velocity Counts
     */
    public static double falconToMPS(double velocitycounts, double circumference, double gearRatio){
        double wheelRPM = falconToRPM(velocitycounts, gearRatio);
        double wheelMPS = (wheelRPM * circumference) / 60;
        return wheelMPS;
    }

     /**
     * @param counts Falcon Counts
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double falconToDegrees(double counts, double gearRatio) {
        return counts * (360.0 / (gearRatio * 2048.0));
    }
    
    /***
     * Converts from degrees to Falcon encoder position
     * 
     * @param degrees Input degrees
     * @param gearRatio Mechanism gear ratio
     * @return Falcon encoder position
     */
    public static double degreesToFalcon(double degrees, double gearRatio){
        double ticks = degrees / (360.0 / (gearRatio * 2048));
        return ticks;
    }

    public static double normalize(final double angle) {
        return (angle >= 0 ? angle : (360 - ((-angle) % 360))) % 360;
    }

}

