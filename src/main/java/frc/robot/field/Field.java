package frc.robot.field;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Field {
  public static final double LENGTH = 16.541;
  public static final double WIDTH = 8.069;
  public static final Translation2d BLUE_HUB = new Translation2d(4.625594, 4.034536);
  public static final Translation2d RED_HUB = new Translation2d(11.915394, 4.034536);
  public static final Translation2d BLUE_OUTPOST = new Translation2d(0.42, 0.7);
  public static final Pose2d BLUE_OUTPOST_POSE =
      new Pose2d(BLUE_OUTPOST, new Rotation2d(Degrees.of(180)));

  public static final Translation2d BLUE_LEFT = new Translation2d(1.218, 7.082);
  public static final Translation2d BLUE_ALLIANCE_LINE = new Translation2d(4, 4);

  public static Translation2d getMyHub() {
    return AllianceFlipUtil.apply(BLUE_HUB);
  }

  public static Translation2d getMyOutpost() {
    return AllianceFlipUtil.apply(BLUE_OUTPOST);
  }

  public static Pose2d getMyOutputPose() {
    return AllianceFlipUtil.apply(BLUE_OUTPOST_POSE);
  }

  public static Translation2d getMyLeft() {
    return AllianceFlipUtil.apply(BLUE_LEFT);
  }

  public static Translation2d getMyAllianceLine() {
    return AllianceFlipUtil.apply(BLUE_ALLIANCE_LINE);
  }
}
