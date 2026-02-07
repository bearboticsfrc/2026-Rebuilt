package bearlib.fms;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.ArrayList;
import java.util.List;

/**
 * Handles the alliance color information for a team during a match. It supports setting and
 * retrieving the alliance, adding listeners for alliance color changes, and notifying these
 * listeners when the alliance color is set or changed.
 */
public class AllianceColor {
  private static Alliance alliance = null;
  private static List<AllianceReadyListener> listeners = new ArrayList<>();

  /**
   * Adds a listener that will be notified when the alliance color changes.
   *
   * @param listener The listener to be added.
   */
  public static void addListener(AllianceReadyListener listener) {
    listeners.add(listener);
  }

  /**
   * Sets the alliance and notifies all registered listeners if the alliance is different from the
   * current alliance.
   *
   * @param newAlliance The new alliance to be set.
   */
  public static void setAllianceColor(Alliance newAlliance) {
    if (alliance == null || alliance != newAlliance) {
      alliance = newAlliance;
      notifyListeners();
    }
  }

  /**
   * Notifies all registered listeners about the current alliance. This method is called internally
   * whenever the alliance is set or changed.
   */
  private static void notifyListeners() {
    for (AllianceReadyListener listener : listeners) {
      listener.updateAlliance(alliance);
    }
  }

  /**
   * Get the current {@link Alliance}.
   *
   * @return The alliance
   */
  public static Alliance getAlliance() {
    return alliance;
  }
}
