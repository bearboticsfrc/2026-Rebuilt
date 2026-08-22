package frc.robot.vision;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.rebuilt.Field;
import frc.robot.vision.VisionSystem.RejectionReason;
import frc.robot.vision.VisionSystem.VisionEstimate;
import java.util.Optional;
import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.LimelightSettings;
import limelight.networktables.LimelightSettings.ImuMode;
import limelight.networktables.LimelightTargetData;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;
import limelight.results.RawFiducial;

public class TurretLimelight {

  private static String LIMELIGHT_NAME = "limelight";

  // Tunable rejection thresholds
  private static final int MIN_TAG_COUNT = 2;
  private static final double MAX_AMBIGUITY = 0.9; // MT1 only, lower = more confident
  private static final double MAX_DISTANCE_M = Units.inchesToMeters(240);
  private static final double MIN_DISTANCE_M = Units.inchesToMeters(12);
  private static final double MAX_POSE_JUMP_M = 2.0; // 0.5; // reject if pose jumps more than this
  private static final double LARGE_VARIANCE = 999999.0;

  private final Limelight limelight;
  private final LimelightPoseEstimator poseEstimator;
  private final EstimationMode mode;

  private static final Matrix<N3, N1> MAX_STD_DEVS = VecBuilder.fill(1e9, 1e9, 1e9);

  private Pose3d lastAcceptedPose = null;

  private Trigger rewindTrigger;

  private Timer rewindTimer = new Timer();

  public TurretLimelight(boolean useMegaTag2) {
    limelight = new Limelight(LIMELIGHT_NAME);
    Limelight.isAvailable(LIMELIGHT_NAME);

    LimelightSettings settings = limelight.getSettings();
    settings.withThrottle(100).withImuMode(ImuMode.SyncInternalImu).save();
    // .withRewindEnable(RewindState.ENABLED)
    mode = useMegaTag2 ? EstimationMode.MEGATAG2 : EstimationMode.MEGATAG1;

    poseEstimator = limelight.createPoseEstimator(mode);

    // rewindTrigger = new Trigger(() -> RobotState.getInstance().isShooting());
    // rewindTrigger
    //     .onTrue(Commands.runOnce(() -> rewindTimer.restart()))
    //     .onFalse(
    //         Commands.runOnce(
    //             () -> {
    //               rewindTimer.stop();
    //               captureVideo(rewindTimer.get() + 1);
    //             }));
  }

  // call on disableInit and enableInit
  public void updateLimelightSettings() {
    if (DriverStation.isDisabled()) {
      LimelightSettings settings = limelight.getSettings();
      settings.withThrottle(100).withImuMode(ImuMode.SyncInternalImu).save();
    } else {
      LimelightSettings settings = limelight.getSettings();
      settings.withThrottle(0).withImuMode(ImuMode.InternalImuExternalAssist).save();
    }
  }

  public void captureVideo(double durationSeconds) {
    LimelightSettings settings = limelight.getSettings();
    settings.rewindCapture(durationSeconds);
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
    POSE_OUT_OF_FIELD,
    DISTANCE_OUT_OF_RANGE,
    POSE_JUMP_TOO_LARGE,
  }

  /**
   * Returns a result every call regardless of acceptance, so callers can log rejection reasons for
   * tuning.
   */
  public VisionEstimate getTurretVisionEstimate() {
    Optional<PoseEstimate> estimate = poseEstimator.getPoseEstimate();

    // No data at all
    if (estimate.isEmpty() || !estimate.get().hasData) {
      return rejected(RejectionReason.NO_DATA);
    }

    PoseEstimate poseEstimate = estimate.get();

    // Not enough tags
    if (poseEstimate.tagCount < MIN_TAG_COUNT) {
      return rejectedWithData(poseEstimate, RejectionReason.INSUFFICIENT_TAGS);
    }

    // RawFiducial[] tags = limelight.getRawFiducial();
    double highestAmbiguity = 0.0;

    // MT1 only: reject high ambiguity (MT2 doesn't have meaningful ambiguity)
    if (mode == EstimationMode.MEGATAG1) {
      if (poseEstimate.getMaxTagAmbiguity() > MAX_AMBIGUITY) {
        return rejectedWithData(poseEstimate, RejectionReason.HIGH_AMBIGUITY);
      }
    }
    // check if the pose is out of the field
    if (Field.poseOutOfField(poseEstimate.pose)) {
      return rejectedWithData(poseEstimate, RejectionReason.POSE_OUT_OF_FIELD);
    }

    // check if we are rotating really fast

    // Distance check - use average tag distance from PoseEstimate
    if (poseEstimate.avgTagDist < MIN_DISTANCE_M || poseEstimate.avgTagDist > MAX_DISTANCE_M) {
      return rejectedWithData(poseEstimate, RejectionReason.DISTANCE_OUT_OF_RANGE);
    }

    // Roll / pitch rejection
    if (Math.abs(Math.toDegrees(poseEstimate.pose.getRotation().getX())) > 5
        || Math.abs(Math.toDegrees(poseEstimate.pose.getRotation().getY())) > 5) {
      return rejectedWithData(poseEstimate, RejectionReason.ROLL_PITCH_REJECTION);
    }

    // Pose jump check - reject if pose moved implausibly far since last accepted
    Pose3d currentPose = poseEstimate.pose;
    if (lastAcceptedPose != null) {
      double jump = currentPose.getTranslation().getDistance(lastAcceptedPose.getTranslation());
      if (jump > MAX_POSE_JUMP_M) {
        return rejectedWithData(poseEstimate, RejectionReason.POSE_JUMP_TOO_LARGE);
      }
    }

    // Accepted
    lastAcceptedPose = currentPose;

    double ambiguity =
        (mode == EstimationMode.MEGATAG1 && poseEstimate.rawFiducials.length > 0)
            ? poseEstimate.rawFiducials[0].ambiguity
            : 0.0;

    latestTags = poseEstimate.rawFiducials;

    // DogLog.log("limelightPose", currentPose);

    return new VisionEstimate(
        currentPose,
        poseEstimate.timestampSeconds,
        calculateStdDevs(poseEstimate),
        poseEstimate.tagCount,
        ambiguity,
        poseEstimate.avgTagDist,
        null // accepted - no rejection reason
        );
  }

  private RawFiducial[] latestTags = new RawFiducial[0];

  public RawFiducial[] getTagList() {
    return latestTags;
  }

  private Matrix<N3, N1> calculateStdDevs(PoseEstimate poseEstimate) {
    double xyStdDevs;
    double rotationStdDevs;

    ChassisSpeeds robotSpeed = Robot.get().getSwerve().getCurrentRobotChassisSpeeds();
    double robotLinearSpeed =
        Math.hypot(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond);
    LimelightTargetData targetData = limelight.getData().targetData;
    double targetSize = targetData.getTargetArea();
    boolean multipleTags = poseEstimate.tagCount > 1;
    double maxAmbiguity = poseEstimate.getMaxTagAmbiguity();
    Pose3d pose = poseEstimate.pose;

    double mt1PoseDifference =
        Robot.get()
            .getSwerve()
            .getRobotPose()
            .getTranslation()
            .getDistance(pose.toPose2d().getTranslation());

    if (robotLinearSpeed <= 0.2 && targetSize > 4) {
      xyStdDevs = 0.1;
      rotationStdDevs = 0.1;
    } else if (multipleTags && targetSize > 2) {
      xyStdDevs = 0.1;
      rotationStdDevs = 0.1;
    } else if (multipleTags && targetSize > 0.2) {
      xyStdDevs = 0.25;
      rotationStdDevs = 8.0;
    } else if (targetSize > 2 && mt1PoseDifference < 0.5) {
      xyStdDevs = 0.5;
      rotationStdDevs = LARGE_VARIANCE;
    } else if (targetSize > 1 && mt1PoseDifference < 0.25) {
      xyStdDevs = 1.0;
      rotationStdDevs = LARGE_VARIANCE;
    } else if (maxAmbiguity < 0.25 && targetSize >= 0.03) {
      xyStdDevs = 1.5;
      rotationStdDevs = LARGE_VARIANCE;
    } else {
      xyStdDevs = LARGE_VARIANCE;
      rotationStdDevs = LARGE_VARIANCE;
    }

    if ((mode == EstimationMode.MEGATAG1) && (poseEstimate.getMaxTagAmbiguity() > 0.5)) {
      rotationStdDevs = Math.max(rotationStdDevs, 50.0);
    }

    if (Math.abs(robotSpeed.omegaRadiansPerSecond) >= 0.5) {
      rotationStdDevs = Math.max(rotationStdDevs, 75.0);
    }

    if (mode == EstimationMode.MEGATAG2) {
      rotationStdDevs = LARGE_VARIANCE;
    }

    Matrix<N3, N1> stdDevs = VecBuilder.fill(xyStdDevs, xyStdDevs, rotationStdDevs);

    return stdDevs;
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
    //  DogLog.log("cameraPose", cameraPose);
    //  DogLog.log("cameraYaw", cameraPose.getRotation().toRotation2d().getDegrees());

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
              Units.inchesToMeters(-6.25), // 6.25 inches back from middle
              Units.inchesToMeters(
                  6.25), // 6.25 inches to the right of middle  Limelight UI uses "LL right" here
              Units.inchesToMeters(24.867407)), // 24+ inches up from the ground
          new Rotation3d());

  private static final Transform3d TURRET_TO_CAMERA =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(6.877436), // out from the center of turret rotation
              Units.inchesToMeters(0.0),
              Units.inchesToMeters(0.0)),
          new Rotation3d(0, Units.degreesToRadians(20), 0)); // 20 degrees pointed up
}
