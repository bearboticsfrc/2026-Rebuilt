package frc.robot.field;

import static edu.wpi.first.units.Units.Degrees;

import bearlib.util.GeomUtil.Zone2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class Field {
  public static final double LENGTH = 16.541;
  public static final double WIDTH = 8.069;
  public static final double ROBOT_WIDTH = 0.85725;
  public static final Translation2d BLUE_HUB = new Translation2d(4.625, 4.034536);
  public static final Translation2d RED_HUB = new Translation2d(11.915394, 4.034536);
  public static final Pose2d BLUE_OUTPOST_POSE =
      new Pose2d(new Translation2d(0.42, 0.7), new Rotation2d(Degrees.of(180)));

  public static final Translation2d BLUE_LEFT = new Translation2d(1.2, 7.0);
  public static final Translation2d BLUE_RIGHT = new Translation2d(1.2, 1);
  public static final Translation2d BLUE_ALLIANCE_LINE = new Translation2d(4, 4);

  public static Zone2d hub =
      new Zone2d(
          new Translation2d(3.987, 4.611),
          new Translation2d(5.229, 4.611),
          new Translation2d(3.987, 3.498),
          new Translation2d(5.229, 3.498));

  public static Pose2d getMyOutputPose() {
    return AllianceFlipUtil.apply(BLUE_OUTPOST_POSE);
  }

  public static Translation2d getMyHub() {
    return AllianceFlipUtil.apply(BLUE_HUB);
  }

  public static Translation3d getMyHub3d() {
    return AllianceFlipUtil.apply(
        new Translation3d(BLUE_HUB.getX(), BLUE_HUB.getY(), Units.inchesToMeters(72.0)));
  }

  public static Translation2d getMyRight() {
    return AllianceFlipUtil.apply(BLUE_RIGHT);
  }

  public static Translation2d getMyLeft() {
    return AllianceFlipUtil.apply(BLUE_LEFT);
  }

  public static Translation2d getMyAllianceLine() {
    return AllianceFlipUtil.apply(BLUE_ALLIANCE_LINE);
  }

  public static Zone2d getMyHubZone() {
    return hub;
  }

  public static boolean poseOutOfField(Pose2d pose2D) {
    double x = pose2D.getX();
    double y = pose2D.getY();
    return (x <= ROBOT_WIDTH || x >= Field.LENGTH - ROBOT_WIDTH)
        || (y <= ROBOT_WIDTH || y >= Field.WIDTH - ROBOT_WIDTH);
  }

  public static boolean poseOutOfField(Pose3d pose3D) {
    return poseOutOfField(pose3D.toPose2d());
  }
}
