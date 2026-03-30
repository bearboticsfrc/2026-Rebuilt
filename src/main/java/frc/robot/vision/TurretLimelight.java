package frc.robot.vision;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.vision.VisionConstants.CULLING_DISTANCE;
import static frc.robot.vision.VisionConstants.MULTI_TAG_STD_DEVS;
import static frc.robot.vision.VisionConstants.SINGLE_TAG_STD_DEVS;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.vision.VisionSystem.RejectionReason;
import frc.robot.vision.VisionSystem.VisionEstimate;
import java.util.Optional;
import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;
import limelight.results.RawFiducial;

public class TurretLimelight {

  private static String LIMELIGHT_NAME = "limelight";

  // Tunable rejection thresholds
  private static final int MIN_TAG_COUNT = 2;
  private static final double MAX_AMBIGUITY = 0.9; // MT1 only, lower = more confident
  private static final double MAX_DISTANCE_M = Units.inchesToMeters(240);
  private static final double MIN_DISTANCE_M = Units.inchesToMeters(24);
  private static final double MAX_POSE_JUMP_M = 2.0; // 0.5; // reject if pose jumps more than this

  private final Limelight limelight;
  private final LimelightPoseEstimator poseEstimator;
  private final EstimationMode mode;

  private static final Matrix<N3, N1> MAX_STD_DEVS =
      VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);

  private Pose3d lastAcceptedPose = null;

  public TurretLimelight(boolean useMegaTag2) {
    limelight = new Limelight(LIMELIGHT_NAME);
    Limelight.isAvailable(LIMELIGHT_NAME);

    mode = useMegaTag2 ? EstimationMode.MEGATAG2 : EstimationMode.MEGATAG1;

    poseEstimator = limelight.createPoseEstimator(mode);
  }

  /**
   * Call once per periodic loop before calling getPoseEstimate(). Only needed for MegaTag2 — sets
   * robot orientation for yaw-locked solve.
   *
   * @param robotYawRads current robot yaw, CCW positive
   * @param turretAngleRads current turret angle, CCW positive from robot forward
   * @param robotAngleVelocityRadsSec current turret angle velocity in radians per second
   */
  public void update(
      double robotYawRads, double turretAngleRads, double robotAngleVelocityRadsSec) {
    // Always update camera offset regardless of mode
    updateCameraOffset(turretAngleRads);

    // MegaTag2 also needs robot orientation
    if (mode == EstimationMode.MEGATAG2) {
      limelight
          .getSettings()
          .withRobotOrientation(
              new Orientation3d(
                  new Rotation3d(0, 0, robotYawRads),
                  new AngularVelocity3d(
                      RadiansPerSecond.zero(),
                      RadiansPerSecond.zero(),
                      RadiansPerSecond.of(robotAngleVelocityRadsSec))));
    }
  }

  public record LimelightResult(
      Pose2d pose,
      double timestampSeconds,
      int tagCount,
      double ambiguity, // 0-1, MT1 only (always 0 for MT2)
      double distanceToTagM,
      RejectionReason rejectionReason // null if accepted
      ) {
    public boolean isAccepted() {
      return rejectionReason == null;
    }
  }

  public enum RejectionReason2 {
    NO_DATA,
    INSUFFICIENT_TAGS,
    HIGH_AMBIGUITY, // MT1 only
    DISTANCE_OUT_OF_RANGE,
    POSE_JUMP_TOO_LARGE,
  }

  /**
   * Returns a result every call regardless of acceptance, so callers can log rejection reasons for
   * tuning.
   */
  public VisionEstimate read() {
    Optional<PoseEstimate> estimate = poseEstimator.getPoseEstimate();

    // No data at all
    if (estimate.isEmpty() || !estimate.get().hasData) {
      return rejected(RejectionReason.NO_DATA);
    }

    PoseEstimate pose = estimate.get();

    // Not enough tags
    if (pose.tagCount < MIN_TAG_COUNT) {
      return rejectedWithData(pose, RejectionReason.INSUFFICIENT_TAGS);
    }

    // MT1 only: reject high ambiguity (MT2 doesn't have meaningful ambiguity)
    if (mode == EstimationMode.MEGATAG1 && pose.rawFiducials.length > 0) {
      double ambiguity = pose.rawFiducials[0].ambiguity;
      if (ambiguity > MAX_AMBIGUITY) {
        return rejectedWithData(pose, RejectionReason.HIGH_AMBIGUITY);
      }
    }

    // Distance check - use average tag distance from PoseEstimate
    if (pose.avgTagDist < MIN_DISTANCE_M || pose.avgTagDist > MAX_DISTANCE_M) {
      return rejectedWithData(pose, RejectionReason.DISTANCE_OUT_OF_RANGE);
    }

    // Pose jump check - reject if pose moved implausibly far since last accepted
    Pose3d currentPose = pose.pose;
    if (lastAcceptedPose != null) {
      double jump = currentPose.getTranslation().getDistance(lastAcceptedPose.getTranslation());
      if (jump > MAX_POSE_JUMP_M) {
        return rejectedWithData(pose, RejectionReason.POSE_JUMP_TOO_LARGE);
      }
    }

    // Accepted
    lastAcceptedPose = currentPose;

    double ambiguity =
        (mode == EstimationMode.MEGATAG1 && pose.rawFiducials.length > 0)
            ? pose.rawFiducials[0].ambiguity
            : 0.0;

    latestTags = pose.rawFiducials;

    return new VisionEstimate(
        currentPose,
        pose.timestampSeconds,
        calculateStdDevs(pose.tagCount, ambiguity, pose.avgTagDist),
        pose.tagCount,
        ambiguity,
        pose.avgTagDist,
        null // accepted - no rejection reason
        );
  }

  private RawFiducial[] latestTags = new RawFiducial[0];

  public RawFiducial[] getTagList() {
    return latestTags;
  }

  private Matrix<N3, N1> calculateStdDevs(int tagCount, double ambiguity, double tagDistance) {

    var estStdDevs = MULTI_TAG_STD_DEVS;

    if (tagCount == 0) {
      // No tags visible.  Return Max Standard Deviations because we don't want to use this
      // measurement
      return MAX_STD_DEVS;
    } else {
      if (tagCount == 1 && tagDistance > CULLING_DISTANCE) return MAX_STD_DEVS;

      // One or more tags visible, run the full heuristic.
      // Increase std devs if only one target is visible
      if (tagCount == 1) estStdDevs = SINGLE_TAG_STD_DEVS;

      // Increase std devs based on (average) distance
      return estStdDevs.times(1 + (tagDistance * tagDistance / 30));
    }
  }

  public void resetLastAcceptedPose() {
    lastAcceptedPose = null;
  }

  // -- private helpers --

  private void updateCameraOffset(double turretAngleRads) {
    Pose3d cameraPose =
        new Pose3d()
            .transformBy(ROBOT_TO_TURRET_PIVOT)
            .transformBy(
                new Transform3d(new Translation3d(), new Rotation3d(0, 0, turretAngleRads)))
            .transformBy(TURRET_TO_CAMERA);

    limelight.getSettings().withCameraOffset(cameraPose);
  }

  private VisionEstimate rejected(RejectionReason reason) {
    return new VisionEstimate(new Pose3d(), 0.0, MAX_STD_DEVS, 0, 0.0, 0.0, reason);
  }

  private VisionEstimate rejectedWithData(PoseEstimate pose, RejectionReason reason) {
    double ambiguity =
        (mode == EstimationMode.MEGATAG1 && pose.rawFiducials.length > 0)
            ? pose.rawFiducials[0].ambiguity
            : 0.0;
    return new VisionEstimate(
        pose.pose,
        pose.timestampSeconds,
        MAX_STD_DEVS,
        pose.tagCount,
        ambiguity,
        pose.avgTagDist,
        reason);
  }

  // Fixed mechanical transforms - tune to match your robot
  private static final Transform3d ROBOT_TO_TURRET_PIVOT =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(6.25),
              Units.inchesToMeters(6.25),
              Units.inchesToMeters(24.867407)),
          new Rotation3d());

  private static final Transform3d TURRET_TO_CAMERA =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(6.877436), Units.inchesToMeters(0.0), Units.inchesToMeters(0.0)),
          new Rotation3d(0, Units.degreesToRadians(20), 0));
}
