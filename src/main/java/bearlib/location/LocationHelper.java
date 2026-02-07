package bearlib.location;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * A utility class providing static methods for geometric calculations related to robot positioning
 * and orientation on the field.
 */
public class LocationHelper {

  /**
   * Calculates the straight-line distance between two poses on the field.
   *
   * @param fromPose The starting pose.
   * @param toPose The target pose.
   * @return The distance between the two poses in meters.
   */
  public static double getDistanceToPose(Pose2d fromPose, Pose2d toPose) {
    return fromPose.getTranslation().getDistance(toPose.getTranslation());
  }

  /**
   * Calculates the rotation offset to a translation (point) on the field.
   *
   * @param fromPose The inital pose.
   * @param translation The target point.
   * @return A Rotation2d representing the angle from the starting pose to the target point.
   */
  public static Rotation2d getRotationToTranslation(Pose2d fromPose, Translation2d translation) {
    return translation.minus(fromPose.getTranslation()).getAngle();
  }
}
