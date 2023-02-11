package lib.utils.drivers;

import com.ctre.phoenix.ErrorCode;
import edu.wpi.first.wpilibj.DriverStation;
public class CTREUtil {
    public interface ConfigCall {
        abstract ErrorCode run();
    }

    private static final int MAX_RETRY_COUNT = 3;
    private static final int RETRY_DELAY_MS = 100;

    public static boolean hasError(ErrorCode err) {
        return err != ErrorCode.OK;
    }

    public static void reportError(ErrorCode err, boolean warning) {
        String msg = String.format("Error: %s", err.toString());
        if (warning) {
            DriverStation.reportWarning(msg, false);
        } else {
            DriverStation.reportError(msg, false);
        }
    }

    public static ErrorCode autoRetry(ConfigCall configCall) {
        ErrorCode err = ErrorCode.GeneralError; // Use an error be default, this will be overridden on the first pass

        // Loop for MAX_RETRY_COUNT
        for (int i = 0; i < MAX_RETRY_COUNT; i++) {
            err = configCall.run();

            // If there is an error, wait for a short period and try again.
            // Otherwise, break the loop
            if(hasError(err)) {
                //DriverStation.reportWarning(String.format("Try #%d failed: %s. Retrying...", i+1, err.toString()), false);
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
            DriverStation.reportError("Failed to configure after " + MAX_RETRY_COUNT + "counts.",  true);
        }
        return err;
    }
}