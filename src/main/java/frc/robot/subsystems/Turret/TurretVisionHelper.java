package frc.robot.subsystems.turret;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import java.util.Map;
import java.util.Optional;
import limelight.Limelight;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.PoseEstimate;

// set throttle in disable/enable
// set imu mode in disable
// set imu mode in enable

public class TurretVisionHelper {

  private final Limelight limelight;
  private final String LIMELIGHT_NAME = "limelight";
  private final LimelightPoseEstimator poseEstimator;

  static final Map<Integer, Transform3d> tagToHubCenterMap =
      Map.ofEntries(
          Map.entry(2, new Transform3d(new Translation3d(-.5842, 0.0, 0.0), new Rotation3d())),
          Map.entry(5, new Transform3d(new Translation3d(-.5842, 0.0, 0.0), new Rotation3d())),
          Map.entry(8, new Transform3d(new Translation3d(-.5842, -.3556, 0.0), new Rotation3d())),
          Map.entry(9, new Transform3d(new Translation3d(-.5842, .3556, 0.0), new Rotation3d())),
          Map.entry(10, new Transform3d(new Translation3d(-.5842, 0.0, 0.0), new Rotation3d())),
          Map.entry(11, new Transform3d(new Translation3d(-.5842, .3556, 0.0), new Rotation3d())),
          Map.entry(18, new Transform3d(new Translation3d(-.5842, 0.0, 0.0), new Rotation3d())),
          Map.entry(21, new Transform3d(new Translation3d(-.5842, 0.0, 0.0), new Rotation3d())),
          Map.entry(24, new Transform3d(new Translation3d(-.5842, -.3556, 0.0), new Rotation3d())),
          Map.entry(25, new Transform3d(new Translation3d(-.5842, .3556, 0.0), new Rotation3d())),
          Map.entry(26, new Transform3d(new Translation3d(-.5842, 0.0, 0.0), new Rotation3d())),
          Map.entry(27, new Transform3d(new Translation3d(-.5842, .3556, 0.0), new Rotation3d())));

  // Fixed mechanical transforms - measure these carefully
  private static final Transform3d ROBOT_TO_TURRET_PIVOT =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(6.25), // forward from robot center
              Units.inchesToMeters(6.25), // lateral from robot center
              Units.inchesToMeters(24.867407) // height of turret pivot
              ),
          new Rotation3d());

  // turret_center -> cameraLens = horizontal:  6.877436", vertical:  6.376978

  private static final Transform3d TURRET_TO_CAMERA =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(6.877436), // camera forward from pivot
              Units.inchesToMeters(0.0), // camera lateral from pivot
              Units.inchesToMeters(0.0) // camera height above pivot
              ),
          new Rotation3d(
              0,
              Units.degreesToRadians(20), // camera pitch down
              0));

  // Known field coordinate of Hub opening center (get X/Y from field drawings)
  // Z = 72" per the game manual
  private static final Translation3d HUB_OPENING_CENTER =
      new Translation3d(
          Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(72.0));

  public TurretVisionHelper() {
    limelight = new Limelight(LIMELIGHT_NAME);

    // Verify it's connected at startup - fires WPILib alert if not
    Limelight.isAvailable(LIMELIGHT_NAME);

    // MegaTag1 - no gyro dependency, purely geometric solve
    poseEstimator = limelight.createPoseEstimator(LimelightPoseEstimator.EstimationMode.MEGATAG1);
  }

  /**
   * Call this every periodic loop iteration. turretAngleRads: CCW positive from robot forward,
   * matching WPILib convention. Negate your Phoenix 6 position if your turret is CW positive.
   */
  public void updateCameraPose(double turretAngleRads) {
    Pose3d cameraPoseRobotSpace =
        new Pose3d()
            .transformBy(ROBOT_TO_TURRET_PIVOT)
            .transformBy(
                new Transform3d(new Translation3d(), new Rotation3d(0, 0, turretAngleRads)))
            .transformBy(TURRET_TO_CAMERA);

    limelight.getSettings().withCameraOffset(cameraPoseRobotSpace);
  }

  /** Result class so callers get everything they need in one place. */
  public record TurretAimResult(
      double yawOffset, // yaw offset to hub center in camera frame
      double distanceMeters, // straight-line distance to Hub center
      int tagCount, // number of tags used in solve
      boolean isValid // false if result should be rejected
      ) {}

  @SuppressWarnings("null")
  public Optional<TurretAimResult> getHubAimOffset() {
    Optional<PoseEstimate> estimate = poseEstimator.getPoseEstimate();

    if (estimate.isEmpty() || !estimate.get().hasData) {
      return Optional.empty();
    }

    PoseEstimate pose = estimate.get();

    // Reject single-tag results - flip ambiguity makes them unreliable
    if (pose.tagCount < 2) {
      return Optional.empty();
    }

    Pose3d cameraPoseTargetSpace = getTargetPoseCameraSpace();

    if (cameraPoseTargetSpace == null) {
      return Optional.empty();
    }

    NetworkTable table = limelight.getNTTable();

    long tid = table.getEntry("tid").getInteger(0);

    Pose3d hubCenter = cameraPoseTargetSpace.plus(tagToHubCenterMap.get(tid));

    double distance = hubCenter.getTranslation().getNorm();

    double radiansOffset = Math.atan(hubCenter.getY() / hubCenter.getX());

    // Reject if distance is implausible - likely a bad solve
    if (distance < Units.inchesToMeters(24) || distance > Units.inchesToMeters(300)) {
      return Optional.empty();
    }

    return Optional.of(new TurretAimResult(radiansOffset, distance, pose.tagCount, true));
  }

  /**
   * Gets camera pose in field space by pulling camerapose_targetspace directly from the JSON
   * result, which is more accurate than reconstructing from bot pose + camera offset.
   */
  //   private Pose3d getCameraPoseFieldSpace(PoseEstimate botPose) {

  //     return limelight.getLatestResults().map(results ->
  // results.camerapose_targetspace).orElse(null);
  //   }

  private Pose3d getTargetPoseCameraSpace() {
    NetworkTable table = limelight.getNTTable();
    double[] rawPose = table.getEntry("targetpose_cameraspace").getDoubleArray(new double[0]);

    // double[] rawPose = table.getEntry("camerapose_targetspace").getDoubleArray(new double[0]);
    long tid = table.getEntry("tid").getInteger(0);

    DogLog.log("camerapose_targetspace", rawPose);
    DogLog.log("tid", tid);
    DogLog.log("angle_to_primary_id", Units.radiansToDegrees(rawPose[5]));

    if (rawPose.length < 6) return null;

    // All zeros means no valid solve
    boolean allZero = true;
    for (double v : rawPose) {
      if (Math.abs(v) > 1e-6) {
        allZero = false;
        break;
      }
    }
    if (allZero) return null;

    return new Pose3d(
        new Translation3d(rawPose[0], rawPose[1], rawPose[2]),
        new Rotation3d(
            Units.degreesToRadians(rawPose[3]),
            Units.degreesToRadians(rawPose[4]),
            Units.degreesToRadians(rawPose[5])));
  }
}
