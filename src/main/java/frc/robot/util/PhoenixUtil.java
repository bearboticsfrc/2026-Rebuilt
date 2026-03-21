package frc.robot.util;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.Supplier;

public class PhoenixUtil {

  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command, String name) {
    StatusCode status = command.get();
    for (int i = 0; i < maxAttempts; i++) {
      if (status.isOK()) break;

      status = command.get();
    }
    if (!status.isOK()) {
      DriverStation.reportError("ERROR Configuring " + name + " motor: " + status, false);
    }
  }
}
