package frc.robot.vision;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;

public class VisionConstants {
  public static final Distance CULLING_DISTANCE = Meters.of(4); // Meters.of(2.5);

  public static final double CULLING_AMBIGUITY = 0.4;

  private static final String FRONT_LEFT_CAMERA_NAME = "OV9281FrontLeft";
  private static final String LUMAP1_CAMERA_NAME = "LUMAP1_OV9281";

  public static final Transform3d ROBOT_TO_FRONT_LEFT_CAMERA =
      new Transform3d(
          new Translation3d(-.272, 0.172, 0.711),
          new Rotation3d(Radians.zero(), Degrees.of(-20), Degrees.zero()));

  public static final Transform3d ROBOT_TO_LUMAP1 =
      new Transform3d(
          new Translation3d(-.20, -0.075, 0.65),
          new Rotation3d(Radians.zero(), Degrees.of(-20), Degrees.zero()));

  public static final VisionCamera FRONT_LEFT_CAMERA =
      new VisionCamera(FRONT_LEFT_CAMERA_NAME, ROBOT_TO_FRONT_LEFT_CAMERA);
  public static final VisionCamera LUMAP1_CAMERA =
      new VisionCamera(LUMAP1_CAMERA_NAME, ROBOT_TO_LUMAP1);

  // The standard deviations of our vision estimated poses, which affect correction rate
  public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(0.001, 0.001, 0.001);
  public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS =
      VecBuilder.fill(0.1, 0.1, Math.toRadians(20));

  public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  public static final AprilTagFieldLayout HUB_TAGS_ONLY_LAYOUT =
      new AprilTagFieldLayout(
          APRIL_TAG_FIELD_LAYOUT.getTags().stream()
              .filter(it -> (it.ID >= 2 && it.ID <= 11) || (it.ID >= 18 && it.ID <= 27))
              .toList(),
          APRIL_TAG_FIELD_LAYOUT.getFieldLength(),
          APRIL_TAG_FIELD_LAYOUT.getFieldWidth());
  public static final AprilTagFieldLayout RED_HUB_TAGS_ONLY_LAYOUT =
      new AprilTagFieldLayout(
          APRIL_TAG_FIELD_LAYOUT.getTags().stream()
              .filter(it -> (it.ID >= 2 && it.ID <= 5 || (it.ID >= 8 && it.ID <= 11)))
              .toList(),
          APRIL_TAG_FIELD_LAYOUT.getFieldLength(),
          APRIL_TAG_FIELD_LAYOUT.getFieldWidth());
  public static final AprilTagFieldLayout BLUE_HUB_TAGS_ONLY_LAYOUT =
      new AprilTagFieldLayout(
          APRIL_TAG_FIELD_LAYOUT.getTags().stream()
              .filter(it -> (it.ID >= 18 && it.ID <= 21 || (it.ID >= 24 && it.ID <= 27)))
              .toList(),
          APRIL_TAG_FIELD_LAYOUT.getFieldLength(),
          APRIL_TAG_FIELD_LAYOUT.getFieldWidth());
}
