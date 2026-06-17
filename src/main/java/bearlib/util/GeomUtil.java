package bearlib.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;

public class GeomUtil {

  /**
   * Represents a 2d area of the field.
   *
   * @param border the zone perimeter, connecting zone verticies
   */
  public record Zone2d(ArrayList<Translation2d> border) {}

  /**
   * Calculates the distance of a pose from a {@link Zone2d}.
   *
   * @param zone the specified {@link Zone2d} on the field
   * @param pose a pose on the field
   * @return double, distance
   */
  public static double getDistanceFromZone(Zone2d zone, Pose2d pose) {
    double dist = Double.MAX_VALUE;

    for (int n = zone.border.size(), i = 0; i < n; i++) {
      // horizontal displacement of border seg
      double xSeg = zone.border.get((i + 1) % n).getX() - zone.border.get((i)).getX();
      // vertical displacement of border seg
      double ySeg = zone.border.get((i + 1) % n).getY() - zone.border.get((i)).getY();
      // horizontal displacement of robot to border seg
      double xBot = pose.getX() - zone.border.get((i)).getX();
      // vertical displacement of robot to border seg
      double yBot = pose.getY() - zone.border.get((i)).getY();

      // % of closest coord along border seg
      double t =
          MathUtil.clamp(((xBot * xSeg) + (yBot * ySeg)) / ((xSeg * xSeg) + (ySeg * ySeg)), 0, 1);

      // closest coord along border seg
      Translation2d c =
          new Translation2d(
              zone.border.get((i)).getX() + (t * xSeg), zone.border.get((i)).getY() + (t * ySeg));

      double d = c.getDistance(pose.getTranslation());
      dist = d <= dist ? d : dist;
    }
    return dist;
  }

  /**
   * Checks if a pose is w/in a {@link Zone2d}.
   *
   * @param zone the specified {@link Zone2d} on the field
   * @param pose a pose on the field
   * @return true, when pose is inside of zone
   */
  public static boolean inZone(Zone2d zone, Pose2d pose) {

    int intersect = 0;

    for (int n = zone.border.size(), i = 0; i < n; i++) {

      double y1 = zone.border.get(i).getY();
      double x1 = zone.border.get(i).getX();
      double y2 = zone.border.get((i + 1) % n).getY();
      double x2 = zone.border.get((i + 1) % n).getX();

      if ((pose.getY() < y1) != (pose.getY() < y2)) {
        double px = (x2 - x1) * (pose.getY() - y1) / (y2 - y1) + x1;
        intersect = (pose.getX() < px) ? intersect + 1 : intersect;
      }
    }
    return intersect % 2 != 0;
  }

  /**
   * Checks if a pose is w/in (distance) meters from zone.
   *
   * @param zone the specified {@link Zone2d} on the field.
   * @param pose a pose on the field
   * @param distance the specified distance from zone
   * @return true, when pose is w/in (distance) meters from zone
   */
  public static boolean withinZone(Zone2d zone, Pose2d pose, double distance) {
    return inZone(zone, pose) ? true : getDistanceFromZone(zone, pose) <= distance;
  }
}
