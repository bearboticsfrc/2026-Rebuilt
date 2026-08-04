package frc.robot.rebuilt;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.Supplier;

public class PhoenixUtil {

  private static final int DEFAULT_MAX_ATTEMPTS = 5;

  public static void applyConfig(Supplier<StatusCode> command, String name) {
    applyConfig(DEFAULT_MAX_ATTEMPTS, command, name);
  }

  public static void applyConfig(int maxAttempts, Supplier<StatusCode> command, String name) {
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < maxAttempts; i++) {
      status = command.get();
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      DriverStation.reportError("ERROR Configuring " + name + " motor: " + status, false);
    }
  }
}
