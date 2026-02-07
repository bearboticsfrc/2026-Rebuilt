/*
 * Initially from https://github.com/HuskieRobotics/3061-lib/blob/main/src/main/java/frc/lib/team6328/util/TunableNumber.java
 * Also from https://github.com/Mechanical-Advantage/RobotCode2025Public/blob/main/src/main/java/org/littletonrobotics/frc2025/util/LoggedTunableNumber.java
 *
 */

package bearlib.util;

import java.util.Arrays;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
 */
public class TunableNumber implements DoubleSupplier {
  private static final String TABLE_KEY = "/Tuning";

  private String key;
  private double defaultValue;
  private double lastHasChangedValue = defaultValue;
  private NetworkNumber dashboardNumber;
  private BooleanSupplier tuningMode;

  /**
   * Create a new TunableNumber
   *
   * @param dashboardKey Key on dashboard
   * @param tuningMode Supplier to indicate tuningMode (return false once values are set)
   */
  public TunableNumber(String dashboardKey, BooleanSupplier tuningMode) {
    this.key = TABLE_KEY + "/" + dashboardKey;
    this.tuningMode = tuningMode;
  }

  /**
   * Create a new TunableNumber with the default value
   *
   * @param dashboardKey Key on dashboard
   * @param defaultValue Default value
   * @param tuningMode Supplier to indicate tuningMode (return false once values are set)
   */
  public TunableNumber(String dashboardKey, double defaultValue, BooleanSupplier tuningMode) {
    this(dashboardKey, tuningMode);
    setDefault(defaultValue);
  }

  /**
   * Get the current value from the dashboard if available and in tuning mode.
   *
   * @return The current value
   */
  public double get() {
    return tuningMode.getAsBoolean() ? dashboardNumber.get() : defaultValue;
  }

  /** Return the value as a double, satisfies the DoubleSupplier interface */
  @Override
  public double getAsDouble() {
    return get();
  }

  /**
   * Get the default value for the number that has been set
   *
   * @return The default value
   */
  public double getDefault() {
    return defaultValue;
  }

  /**
   * Set the default value of the number
   *
   * @param defaultValue The default value
   */
  public void setDefault(double defaultValue) {
    this.defaultValue = defaultValue;
    if (tuningMode.getAsBoolean()) {
      dashboardNumber = new NetworkNumber(key, defaultValue);
    }
  }

  /**
   * Checks whether the number has changed since our last check
   *
   * @return True if the number has changed since the last time this method was called, false
   *     otherwise
   */
  public boolean hasChanged() {
    double currentValue = get();
    if (currentValue != lastHasChangedValue) {
      lastHasChangedValue = currentValue;
      return true;
    }

    return false;
  }

  /**
   * Runs action if any of the tunableNumbers have changed
   *
   * @param action Callback to run when any of the tunable numbers have changed. Access tunable
   *     numbers in order inputted in method
   * @param tunableNumbers All tunable numbers to check
   */
  public static void ifChanged(Consumer<double[]> action, TunableNumber... tunableNumbers) {
    if (Arrays.stream(tunableNumbers).anyMatch(tunableNumber -> tunableNumber.hasChanged())) {
      action.accept(Arrays.stream(tunableNumbers).mapToDouble(TunableNumber::get).toArray());
    }
  }

  /**
   * Runs action if any of the tunableNumbers have changed
   *
   * @param action Callback to run when any of the tunable numbers have changed. Access tunable
   *     numbers in order inputted in method
   * @param tunableNumbers All tunable numbers to check
   */
  public static void ifChanged(Runnable action, TunableNumber... tunableNumbers) {
    ifChanged(values -> action.run(), tunableNumbers);
  }
}
