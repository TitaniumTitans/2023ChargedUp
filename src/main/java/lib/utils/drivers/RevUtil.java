package lib.utils.drivers;


import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj.DriverStation;

public class RevUtil {
    public interface ConfigCall {
        abstract REVLibError run();
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

    public static void autoRetry(ConfigCall configCall) {
        int i;
        REVLibError err;
        for (i = 0; hasError(err = configCall.run()) && i < MAX_RETRY_COUNT; i++) {
            DriverStation.reportWarning(String.format("Try #%d failed: %s. Retrying...", i, err.toString()), false);
            try {
                Thread.sleep(RETRY_DELAY_MS);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
        if (i > MAX_RETRY_COUNT) {
            DriverStation.reportError("Error: Maximum retry count exceeded.", false);
        } else if (i > 0) {
            DriverStation.reportWarning(String.format("Try #%d succeeded!", i), false);
        }
    }
}