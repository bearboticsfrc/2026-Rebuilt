package frc.robot.util;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import java.util.Optional;

public class HubTracker {

  private static Optional<Alliance> autoWinner = getAutoWinner();

  private static final double HUBTRACKER_LOOP_PERIOD = 0.1;

  private Notifier notifier = new Notifier(this::publishToNT);

  public HubTracker() {
    notifier.startPeriodic(HUBTRACKER_LOOP_PERIOD);
  }

  public boolean hubStatus() {
    return isActive();
  }

  public double shiftTimeRemaining() {
    if (!isActive()) return -1;
    return timeRemainingInCurrentShift().orElse(Seconds.of(-1)).in(Seconds);
  }

  public double inactiveShiftTimeRemaining() {
    if (isActive()) return -1;
    return timeRemainingInCurrentShift().orElse(Seconds.of(-1)).in(Seconds);
  }

  public double timeUntilShiftInactive() {
    if (!isActive()) {
      return -1;
    }
    if (isActiveNext()) {
      return timeRemainingInNextShift().orElse(Seconds.of(-1)).in(Seconds);
    } else {
      return shiftTimeRemaining();
    }
  }

  public String shiftName() {
    return getCurrentShift().isEmpty() ? "Not Set" : getCurrentShift().get().toString();
  }

  public double getMatchTimeDouble() {
    return getMatchTime();
  }

  public String getAutoWinnerString() {
    return getAutoWinner().isPresent() ? getAutoWinner().get().toString() : "Not Set";
  }

  private final GenericEntry blueWonAuto =
      Shuffleboard.getTab("Testing")
          .add("Blue Won Auto", false)
          .withWidget(BuiltInWidgets.kToggleButton)
          .getEntry();

  private final GenericEntry redWonAuto =
      Shuffleboard.getTab("Testing")
          .add("Red Won Auto", false)
          .withWidget(BuiltInWidgets.kToggleButton)
          .getEntry();

  private final GenericEntry overrideAutoWinner =
      Shuffleboard.getTab("Testing")
          .add("Override Auto Winnner", false)
          .withWidget(BuiltInWidgets.kToggleButton)
          .getEntry();

  public void blueWonAuto() {
    if (blueWonAuto.getBoolean(true) && autoWinner == null) {
      autoWinner = Optional.of(Alliance.Blue);
    }
    autoWinner = getAutoWinner();
  }

  public void redWonAuto() {
    if (redWonAuto.getBoolean(true) && autoWinner == null) {
      autoWinner = Optional.of(Alliance.Red);
    }
    autoWinner = getAutoWinner();
  }

  public void overrideAutoWinner() {
    if (overrideAutoWinner.getBoolean(true) && autoWinner == Optional.of(Alliance.Blue)) {
      autoWinner = Optional.of(Alliance.Red);
    } else if (overrideAutoWinner.getBoolean(true) && autoWinner == Optional.of(Alliance.Red)) {
      autoWinner = Optional.of(Alliance.Blue);
    } else {
      autoWinner = getAutoWinner();
    }
  }

  public boolean inEndGame() {
    return getCurrentShift() == Optional.of(Shift.ENDGAME);
  }

  /**
   * Returns an {@link Optional} containing the current {@link Shift}. Will return {@link
   * Optional#empty()} if disabled or in between auto and teleop.
   */
  public static Optional<Shift> getCurrentShift() {
    double matchTime = getMatchTime();
    if (matchTime < 0) return Optional.empty();

    for (Shift shift : Shift.values()) {
      if (matchTime < shift.endTime) {
        return Optional.of(shift);
      }
    }
    return Optional.empty();
  }

  /**
   * Returns an {@link Optional} containing the current {@link Time} remaining in the current shift.
   * Will return {@link Optional#empty()} if disabled or in between auto and teleop.
   */
  public static Optional<Time> timeRemainingInCurrentShift() {
    return getCurrentShift().map((shift) -> Seconds.of(shift.endTime - getMatchTime()));
  }

  public static Optional<Time> timeRemainingInNextShift() {
    return getNextShift().map((shift) -> Seconds.of(shift.endTime - getMatchTime()));
  }

  /**
   * Returns an {@link Optional} containing the next {@link Shift}. Will return {@link
   * Optional#empty()} if disabled or in between auto and teleop.
   */
  public static Optional<Shift> getNextShift() {
    double matchTime = getMatchTime();

    for (Shift shift : Shift.values()) {
      if (matchTime < shift.startTime) {
        return Optional.of(shift);
      }
    }
    return Optional.empty();
  }

  /**
   * Returns whether the hub is active during the specified {@link Shift} for the specified {@link
   * Alliance}. Will return {@code false} if disabled or in between auto and teleop.
   */
  public static boolean isActive(Alliance alliance, Shift shift) {
    switch (shift.activeType) {
      case BOTH:
        return true;
      case AUTO_WINNER:
        return autoWinner.isPresent() && autoWinner.get() == alliance;
      case AUTO_LOSER:
        return autoWinner.isPresent() && autoWinner.get() != alliance;
      default:
        return false;
    }
  }

  /**
   * Returns whether the hub is active during the current {@link Shift} for the specified {@link
   * Alliance}. Will return {@code false} if disabled or in between auto and teleop.
   */
  public static boolean isActive(Alliance alliance) {
    Optional<Shift> currentShift = getCurrentShift();
    return currentShift.isPresent() && isActive(alliance, currentShift.get());
  }

  /**
   * Returns whether the hub is active during the specified {@link Shift} for the robot's {@link
   * Alliance}. Will return {@code false} if disabled or in between auto and teleop.
   */
  public static boolean isActive(Shift shift) {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    return alliance.isPresent() && isActive(alliance.get(), shift);
  }

  /**
   * Returns whether the hub is active during the current {@link Shift} for the robot's {@link
   * Alliance}. Will return {@code false} if disabled or in between auto and teleop.
   */
  public static boolean isActive() {
    Optional<Shift> currentShift = getCurrentShift();
    Optional<Alliance> alliance = DriverStation.getAlliance();
    return currentShift.isPresent()
        && alliance.isPresent()
        && isActive(alliance.get(), currentShift.get());
  }

  /**
   * Returns whether the hub is active for the next {@link Shift} for the specified {@link
   * Alliance}. Will return {@code false} if disabled or in between auto and teleop.
   */
  public static boolean isActiveNext(Alliance alliance) {
    Optional<Shift> nextShift = getNextShift();
    return nextShift.isPresent() && isActive(alliance, nextShift.get());
  }

  /**
   * Returns whether the hub is active during the specified {@link Shift} for the specified {@link
   * Alliance}. Will return {@code false} if disabled or in between auto and teleop.
   */
  public static boolean isActiveNext() {
    Optional<Shift> nextShift = getNextShift();
    Optional<Alliance> alliance = DriverStation.getAlliance();
    return nextShift.isPresent()
        && alliance.isPresent()
        && isActive(alliance.get(), nextShift.get());
  }

  /**
   * Returns the {@link Alliance} that won auto as specified by the FMS/Driver Station's game
   * specific message data. Will return {@link Optional#empty()} if no game message or alliance is
   * available.
   */
  public static Optional<Alliance> getAutoWinner() {
    String msg = DriverStation.getGameSpecificMessage();
    char msgChar = msg.length() > 0 ? msg.charAt(0) : ' ';
    switch (msgChar) {
      case 'B':
        return Optional.of(Alliance.Blue);
      case 'R':
        return Optional.of(Alliance.Red);
      default:
        return Optional.empty();
    }
  }

  /**
   * Counts up from 0 to 160 seconds as match progresses. Returns -1 if not match isn't running or
   * if in between auto and teleop
   */
  public static double getMatchTime() {
    if (DriverStation.isAutonomous()) {
      if (DriverStation.getMatchTime() < 0) return DriverStation.getMatchTime();
      return 20 - DriverStation.getMatchTime();
    } else if (DriverStation.isTeleop()) {
      if (DriverStation.getMatchTime() < 0) return DriverStation.getMatchTime();
      return 160 - DriverStation.getMatchTime();
    }
    return -1;
  }

  /**
   * Represents an alliance shift.<br>
   *
   * <h4>Values:</h4>
   *
   * <ul>
   *   <li>{@link Shift#AUTO} (0-20 sec)
   *   <li>{@link Shift#TRANSITION} (20-30 sec)
   *   <li>{@link Shift#SHIFT_1} (30-55 sec)
   *   <li>{@link Shift#SHIFT_2} (55-80 sec)
   *   <li>{@link Shift#SHIFT_3} (80-105 sec)
   *   <li>{@link Shift#SHIFT_4} (105-130 sec)
   *   <li>{@link Shift#ENDGAME} (130-160 sec)
   * </ul>
   */
  public enum Shift {
    AUTO(0, 20, ActiveType.BOTH),
    TRANSITION(20, 30, ActiveType.BOTH),
    SHIFT_1(30, 55, ActiveType.AUTO_LOSER),
    SHIFT_2(55, 80, ActiveType.AUTO_WINNER),
    SHIFT_3(80, 105, ActiveType.AUTO_LOSER),
    SHIFT_4(105, 130, ActiveType.AUTO_WINNER),
    ENDGAME(130, 160, ActiveType.BOTH),
    ENDGAME_NULL_PROTECT(160, 160, ActiveType.BOTH);

    final int startTime;
    final int endTime;
    final ActiveType activeType;

    private Shift(int startTime, int endTime, ActiveType activeType) {
      this.startTime = startTime;
      this.endTime = endTime;
      this.activeType = activeType;
    }
  }

  private enum ActiveType {
    BOTH,
    AUTO_WINNER,
    AUTO_LOSER
  }

  public void publishToNT() {
    String ntKey = "Robot/tracker";
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    nt.getEntry(ntKey + "/hubStatus").setBoolean(hubStatus());
    nt.getEntry(ntKey + "/In Endgame").setBoolean(inEndGame());
    nt.getEntry(ntKey + "/Time Left in Active Shift").setDouble(shiftTimeRemaining());
    nt.getEntry(ntKey + "/Time Left in Inactive Shift").setDouble(inactiveShiftTimeRemaining());
    nt.getEntry(ntKey + "/Seconds until inactive").setDouble(timeUntilShiftInactive());
    nt.getEntry(ntKey + "/Shift").setString(shiftName());
    nt.getEntry(ntKey + "/Match Time").setDouble(getMatchTimeDouble());
    nt.getEntry(ntKey + "/Auto Winner").setString(getAutoWinnerString());
  }
}
