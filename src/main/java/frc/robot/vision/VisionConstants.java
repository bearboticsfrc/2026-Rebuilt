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
  public static final Distance CULLING_DISTANCE = Meters.of(2.2); // Meters.of(2.5);

  public static final double CULLING_AMBIGUITY = 0.2;

  public static final String FRONT_CAMERA_NAME = "ThriftyFront";
  public static final String REAR_CAMERA_NAME = "ThriftyRear";
  public static final String LEFT_CAMERA_NAME = "ThriftyLeft";
  public static final String RIGHT_CAMERA_NAME = "ThriftyRight";

  public static final Transform3d ROBOT_TO_FRONT_CAMERA =
      new Transform3d(
          new Translation3d(Inches.of(12.53), Inches.of(7.5), Inches.of(22.81)),
          new Rotation3d(Radians.zero(), Degrees.of(-15), Degrees.zero()));

  public static final Transform3d ROBOT_TO_REAR_CAMERA =
      new Transform3d(
          new Translation3d(Inches.of(-12.53), Inches.of(8.0), Inches.of(16.81)),
          new Rotation3d(Radians.zero(), Degrees.of(-20), Degrees.of(180)));

  public static final Transform3d ROBOT_TO_LEFT_CAMERA =
      new Transform3d(
          new Translation3d(Inches.of(6.5), Inches.of(12.53), Inches.of(22.81)),
          new Rotation3d(Radians.zero(), Degrees.of(-15), Degrees.of(90)));

  public static final Transform3d ROBOT_TO_RIGHT_CAMERA =
      new Transform3d(
          new Translation3d(Inches.of(7.5), Inches.of(-12.53), Inches.of(22.81)),
          new Rotation3d(Radians.zero(), Degrees.of(-20), Degrees.of(-90)));

  public static final VisionCamera FRONT_CAMERA =
      new VisionCamera(FRONT_CAMERA_NAME, ROBOT_TO_FRONT_CAMERA);

  public static final VisionCamera REAR_CAMERA =
      new VisionCamera(REAR_CAMERA_NAME, ROBOT_TO_REAR_CAMERA);

  public static final VisionCamera LEFT_CAMERA =
      new VisionCamera(LEFT_CAMERA_NAME, ROBOT_TO_LEFT_CAMERA);

  public static final VisionCamera RIGHTT_CAMERA =
      new VisionCamera(RIGHT_CAMERA_NAME, ROBOT_TO_RIGHT_CAMERA);

  // The standard deviations of our vision estimated poses, which affect correction rate
  public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(0.4, 0.4, 2.0);
  public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.1, 0.1, 0.5);

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
