package lib.utils.drivers;


import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.DriverStation;
public class RevUtil {
    public interface ConfigCall {
        REVLibError run();
    }

    private static final int MAX_RETRY_COUNT = 3;
    private static final int RETRY_DELAY_MS = 10;

    public static boolean hasError(REVLibError err) {
        return err != REVLibError.kOk;
    }

    public static void reportError(REVLibError err, boolean warning) {
        String msg = String.format("Error: %s", err.toString());
        if (warning) {
            DriverStation.reportWarning(msg, false);
        } else {
            DriverStation.reportError(msg, false);
        }
    }

    public static REVLibError autoRetry(ConfigCall configCall) {
        REVLibError err = REVLibError.kOk; // Use an error be default, this will be overridden on the first pass

        // Loop for MAX_RETRY_COUNT
        for (int i = 0; i < MAX_RETRY_COUNT; i++) {
            err = configCall.run();

            // If there is an error, wait for a short period and try again.
            // Otherwise, break the loop
            if(hasError(err)) {
                try {
                    // Some tools recommended against this, but because we are dealing with hardware, this should be sufficient.
                    Thread.sleep(RETRY_DELAY_MS);
                } catch (InterruptedException ignored) {
                    // For some reason, the thread was intentionally interrupted
                    // Follow through with the interrupt
                    Thread.currentThread().interrupt();
                }
            } else {
                break;
            }
        }

        // If err is still an error, then the loop reached MAX_RETRY_COUNT and could not set the config
        if (hasError(err)) {
            // Display the error to the driver with stack trace to see WHAT failed to configure.
            DriverStation.reportError("Failed to configure after " + MAX_RETRY_COUNT + "counts. (" + configCall.hashCode() + ")",  true);
        }
        return err;
    }
}