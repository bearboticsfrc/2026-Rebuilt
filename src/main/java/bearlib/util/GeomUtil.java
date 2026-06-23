package bearlib.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class GeomUtil {

  /**
   * Represents a 2d area of the field.
   *
   * @param coords connecting zone verticies
   */
  public static record Zone2d(Translation2d... coords) {
    public Zone2d(Translation2d... coords) {
      this.coords = coords;
    }
  }

  /**
   * Calculates the distance of a pose from a {@link Zone2d}.
   *
   * @param zone the specified {@link Zone2d} on the field
   * @param pose a pose on the field
   * @return double, distance
   */
  public static double getDistanceFromZone(Zone2d zone, Pose2d pose) {
    double dist = Double.MAX_VALUE;

    for (int n = zone.coords.length, i = 0; i < n; i++) {
      // horizontal displacement of zone seg
      double xSeg = zone.coords[(i + 1) % n].getX() - zone.coords[i].getX();
      // vertical displacement of zone seg
      double ySeg = zone.coords[(i + 1) % n].getY() - zone.coords[i].getY();
      // horizontal displacement of robot to zone seg
      double xBot = pose.getX() - zone.coords[i].getX();
      // vertical displacement of robot to zone seg
      double yBot = pose.getY() - zone.coords[i].getY();

      // % of closest coord along zone seg
      double t =
          MathUtil.clamp(((xBot * xSeg) + (yBot * ySeg)) / ((xSeg * xSeg) + (ySeg * ySeg)), 0, 1);

      // closest coord along zone seg
      Translation2d c =
          new Translation2d(zone.coords[i].getX() + (t * xSeg), zone.coords[i].getY() + (t * ySeg));

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

    for (int n = zone.coords.length, i = 0; i < n; i++) {

      double y1 = zone.coords[i].getY();
      double x1 = zone.coords[i].getX();
      double y2 = zone.coords[(i + 1) % n].getY();
      double x2 = zone.coords[(i + 1) % n].getX();

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
