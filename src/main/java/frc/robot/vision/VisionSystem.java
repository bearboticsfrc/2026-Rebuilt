package frc.robot.vision;

import static frc.robot.vision.VisionConstants.*;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Robot;
import frc.robot.field.Field;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.spectrumLib.vision.Limelight;
import frc.spectrumLib.vision.Limelight.LimelightConfig;
import frc.spectrumLib.vision.LimelightHelpers.RawFiducial;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSystem {
  private static final double VISION_LOOP_PERIOD = 0.02;
  private static final double MAX_TIME_DELTA_SECONDS = 0.1;
  private static final double LARGE_VARIANCE = 999999.0;

  private final List<PhotonCamera> cameras = new ArrayList<>();
  private final List<PhotonPoseEstimator> photonEstimators = new ArrayList<>();
  @Getter public final Limelight turretLimelight;
  public static final String limelight = "limelight";
  public static final int limelightPipelineIndex = 0;

  @Getter
  final Translation2d robotToTurretCenter =
      new Translation2d(Units.inchesToMeters(-6.25), Units.inchesToMeters(6.25));

  // 24.867 z

  @Getter
  final Translation2d turretCenterToCamera = new Translation2d(Units.inchesToMeters(6.877436), 0);

  @Getter private final DoubleSupplier turretRotationSupplier;

  public record VisionEstimate(
      Pose3d pose, double timestampSeconds, Matrix<N3, N1> stdDevs, int numTags) {}
  ;

  private static final Matrix<N3, N1> MAX_STD_DEVS =
      VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
  private Matrix<N3, N1> curStdDevs;

  // Simulation
  private PhotonCameraSim cameraSim;
  private VisionSystemSim visionSim;
  private Notifier simNotifier = null;
  private static final double simLoopPeriod = 0.005; // 5ms

  @Logged(name = "Camera Poses", importance = Importance.CRITICAL)
  public Map<String, Pose2d> latestCameraPose = new HashMap<String, Pose2d>();

  @Logged(name = "Target Poses", importance = Importance.CRITICAL)
  public final List<Pose2d> targetPoses = new ArrayList<>();

  private final CommandSwerveDrivetrain drivetrain;

  private final Notifier poseEstimationNotifier = new Notifier(this::poseEstimationPeriodic);

  @Logged
  public String getLimelightLogMessage() {
    return turretLimelight.getLogStatus();
  }

  @Getter
  final LimelightConfig turretConfig =
      new LimelightConfig(limelight)
          .withTranslation(
              Units.inchesToMeters(0.430), Units.inchesToMeters(-5.06), Units.inchesToMeters(27))
          .withRotation(0, 10, 180);

  public VisionSystem(
      List<VisionCamera> visionCameras,
      CommandSwerveDrivetrain drivetrain,
      DoubleSupplier turretRotationSupplier) {
    this.drivetrain = drivetrain;
    this.turretRotationSupplier = turretRotationSupplier;
    if (Robot.isSimulation()) {
      setupSimulation();
    }
    for (VisionCamera visionCamera : visionCameras) {
      PhotonCamera camera = new PhotonCamera(visionCamera.getName());

      PhotonPoseEstimator photonEstimator =
          new PhotonPoseEstimator(APRIL_TAG_FIELD_LAYOUT, visionCamera.getTransform());

      cameras.add(camera);
      photonEstimators.add(photonEstimator);

      // ----- Simulation
      if (Robot.isSimulation()) {
        addCameraToSim(camera, visionCamera.getTransform());
      }
    }

    turretLimelight = new Limelight("limelight", limelightPipelineIndex, turretConfig);

    poseEstimationNotifier.startPeriodic(VISION_LOOP_PERIOD);
  }

  private void poseEstimationPeriodic() {
    List<VisionEstimate> visionEstimates = getEstimatedGlobalPoses();

    integrateMultipleEstimates(visionEstimates);

    // for (VisionEstimate visionEstimate : visionEstimates) {
    //   drivetrain.addVisionMeasurement(
    //       visionEstimate.pose.toPose2d(),
    //       Utils.fpgaToCurrentTime(visionEstimate.timestampSeconds),
    //       visionEstimate.stdDevs());
    // }
  }

  /**
   * Returns the latest standard deviations of the estimated pose from {@link
   * #getEstimatedGlobalPose()}, This should only be used when there are targets visible.
   */
  public Matrix<N3, N1> getEstimationStdDevs() {
    return curStdDevs;
  }

  public void resetToFrontCameraPose() {
    Pose2d visionPose = latestCameraPose.get(VisionConstants.FRONT_CAMERA_NAME);
    if (visionPose != null) {
      drivetrain.resetPose(visionPose);
    } else {
      System.out.println("Tried to reset pose from front camera without a pose!!!!!!!!!!!!!");
    }
  }

  /**
   * The latest estimated robot pose on the field from vision data. This may be empty. This should
   * only be called once per loop.
   *
   * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
   *     used for estimation.
   */
  public List<VisionEstimate> getEstimatedGlobalPoses() {
    targetPoses.clear();

    List<VisionEstimate> visionEstimates = new ArrayList<>();

    for (PhotonCamera camera : cameras) {
      List<PhotonPipelineResult> changes = camera.getAllUnreadResults();
      if (changes.isEmpty()) continue;

      // we only care about the last result from this camera
      PhotonPipelineResult change = changes.get(changes.size() - 1);
      PhotonPoseEstimator photonPoseEstimator = photonEstimators.get(cameras.indexOf(camera));
      Optional<EstimatedRobotPose> visionEstimation =
          photonPoseEstimator.estimateCoprocMultiTagPose(change);

      // If there is no multi tag pose, revert to lowest ambiguity of single tag
      if (visionEstimation.isEmpty()) {
        visionEstimation = photonPoseEstimator.estimateLowestAmbiguityPose(change);
      }
      if (visionEstimation.isEmpty()) {
        continue;
      }
      if (isTooAmbiguous(visionEstimation.get()) || isTooFar(visionEstimation.get())) {
        continue;
      }

      if (shouldReject(
          visionEstimation.get().estimatedPose.toPose2d(),
          visionEstimation.get().targetsUsed.get(0).area)) {
        continue;
      }

      updatedTargetPoses(visionEstimation.get().targetsUsed);

      if (Robot.isSimulation()) {
        visionEstimation.ifPresentOrElse(
            est ->
                getSimDebugField()
                    .getObject("VisionEstimation")
                    .setPose(est.estimatedPose.toPose2d()),
            () -> {
              // if (newResult)
              getSimDebugField().getObject("VisionEstimation").setPoses();
            });
      }

      Matrix<N3, N1> stdDevs =
          calculateStdDevs(photonPoseEstimator, visionEstimation, change.getTargets());
      visionEstimates.add(
          new VisionEstimate(
              visionEstimation.get().estimatedPose,
              visionEstimation.get().timestampSeconds,
              stdDevs,
              visionEstimation.get().targetsUsed.size()));

      latestCameraPose.put(camera.getName(), visionEstimation.get().estimatedPose.toPose2d());
    }

    // Optional<VisionEstimate> turretEstimate = getTurretPose();
    // if (turretEstimate.isPresent()) {
    //   visionEstimates.add(turretEstimate.get());
    //   updatedTargetPosesFromLimelight(turretLimelight.getRawFiducial());
    // }

    return visionEstimates;
  }

  public Optional<VisionEstimate> getTurretPose() {

    if (!turretLimelight.targetInView()) {
      turretLimelight.setTagStatus("No Targets in View");
      turretLimelight.sendInvalidStatus("No Targets in View Rejection");
      return Optional.empty();
    }

    boolean multiTags = turretLimelight.multipleTagsInView();
    double targetSize = turretLimelight.getTargetSize();
    Pose3d megaTag1Pose3d = turretLimelight.getMegaTag1_Pose3d();
    Pose2d megaTag1Pose2d = megaTag1Pose3d.toPose2d();
    RawFiducial[] tags = turretLimelight.getRawFiducial();

    double highestAmbiguity = 0.0;
    ChassisSpeeds robotSpeed = Robot.get().getSwerve().getCurrentRobotChassisSpeeds();
    double robotLinearSpeed =
        Math.hypot(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond);

    double mt1PoseDifference =
        Robot.get()
            .getSwerve()
            .getRobotPose()
            .getTranslation()
            .getDistance(megaTag1Pose2d.getTranslation());

    /* ---------------- Rejections ---------------- */
    turretLimelight.setTagStatus("");
    for (RawFiducial tag : tags) {
      highestAmbiguity = Math.max(highestAmbiguity, tag.ambiguity);

      if (tag.ambiguity > 0.9) {
        turretLimelight.sendInvalidStatus("High Ambiguity Rejection");
        return Optional.empty();
      }
    }

    if (shouldReject(megaTag1Pose2d, targetSize)) {
      turretLimelight.sendInvalidStatus("Generic Rejection");
      return Optional.empty();
    }

    // Roll / pitch rejection
    if (Math.abs(Math.toDegrees(megaTag1Pose3d.getRotation().getX())) > 5
        || Math.abs(Math.toDegrees(megaTag1Pose3d.getRotation().getY())) > 5) {
      turretLimelight.sendInvalidStatus("Roll/Pitch Rejection");
      return Optional.empty();
    }

    /* ---------------- Integration tuning ---------------- */
    double xyStds;
    double degStds;

    if (robotLinearSpeed <= 0.2 && targetSize > 4) {
      turretLimelight.sendValidStatus("Stationary close integration");
      xyStds = 0.1;
      degStds = 0.1;
    } else if (multiTags && targetSize > 2) {
      turretLimelight.sendValidStatus("Strong multi integration");
      xyStds = 0.1;
      degStds = 0.1;
    } else if (multiTags && targetSize > 0.2) {
      turretLimelight.sendValidStatus("Multi integration");
      xyStds = 0.25;
      degStds = 8.0;
    } else if (targetSize > 2 && mt1PoseDifference < 0.5) {
      turretLimelight.sendValidStatus("Close integration");
      xyStds = 0.5;
      degStds = LARGE_VARIANCE;
    } else if (targetSize > 1 && mt1PoseDifference < 0.25) {
      turretLimelight.sendValidStatus("Proximity integration");
      xyStds = 1.0;
      degStds = LARGE_VARIANCE;
    } else if (highestAmbiguity < 0.25 && targetSize >= 0.03) {
      turretLimelight.sendValidStatus("Stable integration");
      xyStds = 1.5;
      degStds = LARGE_VARIANCE;
    } else {
      turretLimelight.sendInvalidStatus("Confidence too low");
      return Optional.empty();
    }

    /* ---------------- MT1-specific tightening ---------------- */
    // MT1 rotation is weak — trust it even less when ambiguity rises
    if (highestAmbiguity > 0.5) {
      degStds = Math.max(degStds, 50.0);
    }

    if (Math.abs(robotSpeed.omegaRadiansPerSecond) >= 0.5) {
      degStds = Math.max(degStds, 75.0);
    }

    // if (!integrateXY) {
    //   xyStds = config.getKLargeVariance();
    // }

    // If we're forcing integration, use very tight stds
    // if (forceIntegration) {
    //   xyStds = 0.01;
    //   degStds = 0.01;
    // }

    /* ---------------- Turret adjustment ---------------- */
    double turretDegrees = turretRotationSupplier.getAsDouble();
    Rotation2d turretRotation = Rotation2d.fromDegrees(turretDegrees);
    // flip another 180 ?

    // Rotate vector by turret angle
    Translation2d turretToRotatedCamera = turretCenterToCamera.rotateBy(turretRotation);

    // Add turret center offset to get full robot->camera vector
    Translation2d robotToRotatedCamera = getRobotToTurretCenter().plus(turretToRotatedCamera);

    Translation2d robotToCameraField =
        robotToRotatedCamera.rotateBy(Robot.get().getSwerve().getRobotPose().getRotation());

    Translation2d robotTranslation = megaTag1Pose2d.getTranslation().minus(robotToCameraField);
    Rotation2d robotRotation = megaTag1Pose2d.getRotation().minus(turretRotation);

    Pose2d integratedPose = new Pose2d(robotTranslation, robotRotation);

    double timestamp = Utils.fpgaToCurrentTime(turretLimelight.getMegaTag1PoseTimestamp());
    Matrix<N3, N1> stdDevs = VecBuilder.fill(xyStds, xyStds, degStds);
    int numTags = tags.length;

    return Optional.of(new VisionEstimate(new Pose3d(integratedPose), timestamp, stdDevs, numTags));
  }

  /**
   * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
   * deviations based on number of tags, estimation strategy, and distance from the tags.
   *
   * @param photonEstimator The photon pose estimator to use.
   * @param estimatedPose The estimated pose to guess standard deviations for.
   * @param targets All targets in this camera frame
   */
  private Matrix<N3, N1> calculateStdDevs(
      PhotonPoseEstimator photonEstimator,
      Optional<EstimatedRobotPose> estimatedPose,
      List<PhotonTrackedTarget> targets) {

    var estStdDevs = MULTI_TAG_STD_DEVS;
    int numTags = 0;
    double avgDist = 0;

    // Precalculation - see how many tags we found, and calculate an average-distance metric
    for (PhotonTrackedTarget tgt : targets) {
      Optional<Pose3d> tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty()) continue;
      numTags++;
      avgDist +=
          tagPose
              .get()
              .toPose2d()
              .getTranslation()
              .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
    }

    if (numTags == 0) {
      // No tags visible.  Return Max Standard Deviations because we don't want to use this
      // measurement
      return MAX_STD_DEVS;
    } else {
      avgDist /= numTags;
      if (numTags == 1 && avgDist > CULLING_DISTANCE) return MAX_STD_DEVS;

      // One or more tags visible, run the full heuristic.
      // Increase std devs if only one target is visible
      if (numTags == 1) estStdDevs = SINGLE_TAG_STD_DEVS;

      // Increase std devs based on (average) distance
      return estStdDevs.times(1 + (avgDist * avgDist / 30));
    }
  }

  private void updatedTargetPoses(List<PhotonTrackedTarget> targetList) {
    for (PhotonTrackedTarget trackedTarget : targetList) {
      int fiducialId = trackedTarget.getFiducialId();
      Pose3d tagPose = VisionConstants.APRIL_TAG_FIELD_LAYOUT.getTagPose(fiducialId).get();

      targetPoses.add(tagPose.toPose2d());
    }
  }

  private void updatedTargetPosesFromLimelight(RawFiducial[] tags) {
    for (RawFiducial tag : tags) {
      int fiducialId = tag.id;
      Pose3d tagPose = VisionConstants.APRIL_TAG_FIELD_LAYOUT.getTagPose(fiducialId).get();

      targetPoses.add(tagPose.toPose2d());
    }
  }

  private boolean isTooFar(EstimatedRobotPose estimate) {
    return estimate.targetsUsed.get(0).getBestCameraToTarget().getTranslation().getNorm()
        > CULLING_DISTANCE;
  }

  private boolean isTooAmbiguous(EstimatedRobotPose estimate) {
    return estimate.targetsUsed.get(0).poseAmbiguity > CULLING_AMBIGUITY;
  }

  private boolean shouldReject(Pose2d pose, double targetSize) {
    /* rejections */
    if (Field.poseOutOfField(pose)) {
      return true;
    }

    if (Math.abs(Robot.get().getSwerve().getCurrentRobotChassisSpeeds().omegaRadiansPerSecond)
        >= 1.6) {
      return true;
    }

    // Final check, if it's small reject, else return false and integrate
    return targetSize <= 0.025;
  }

  /** Helper to integrate multiple estimates close in time by fusing them together first */
  public void integrateMultipleEstimates(List<VisionSystem.VisionEstimate> list) {
    // Collect non-null estimates
    // List<VisionEstimate> list = new ArrayList<>();
    // for (VisionEstimate e : estimates) {
    //   if (e != null) list.add(e);
    // }
    if (list.isEmpty()) return;

    // Sort by timestamp ascending (old -> new). fuseEstimates expects to project
    // older -> newer.
    list.sort(Comparator.comparingDouble(VisionEstimate::timestampSeconds));

    // Iteratively group/fuse estimates close in time.
    VisionEstimate currentGroup = list.get(0);
    for (int i = 1; i < list.size(); i++) {
      VisionEstimate next = list.get(i);
      double timeDelta = Math.abs(next.timestampSeconds() - currentGroup.timestampSeconds());

      if (timeDelta < MAX_TIME_DELTA_SECONDS) {
        // Fuse into current group (currentGroup older, next newer)
        currentGroup = fuseEstimates(currentGroup, next);
      } else {
        // No close timestamp: integrate current group and start a new one
        integrateSingleEstimate(currentGroup);
        currentGroup = next;
      }
    }
    // integrate the final fused group
    integrateSingleEstimate(currentGroup);
  }

  /** Fuses two vision pose estimates using inverse-variance weighting. (FRC254 2025) */
  private VisionEstimate fuseEstimates(VisionEstimate a, VisionEstimate b) {
    // Ensure b is the newer measurement
    if (b.timestampSeconds() < a.timestampSeconds()) {
      VisionEstimate tmp = a;
      a = b;
      b = tmp;
    }

    // Project both estimates to the same timestamp using odometry
    Transform2d a_T_b =
        Robot.get()
            .getSwerve()
            .getPoseAtTimestamp(b.timestampSeconds())
            .minus(Robot.get().getSwerve().getPoseAtTimestamp(a.timestampSeconds()));

    Pose2d poseA = a.pose().toPose2d().transformBy(a_T_b);
    Pose2d poseB = b.pose().toPose2d();

    // Inverse‑variance weighting
    var varianceA = a.stdDevs().elementTimes(a.stdDevs());
    var varianceB = b.stdDevs().elementTimes(b.stdDevs());

    Rotation2d fusedHeading = poseB.getRotation();
    if (varianceA.get(2, 0) < LARGE_VARIANCE && varianceB.get(2, 0) < LARGE_VARIANCE) {
      fusedHeading =
          new Rotation2d(
              poseA.getRotation().getCos() / varianceA.get(2, 0)
                  + poseB.getRotation().getCos() / varianceB.get(2, 0),
              poseA.getRotation().getSin() / varianceA.get(2, 0)
                  + poseB.getRotation().getSin() / varianceB.get(2, 0));
    }

    double weightAx = 1.0 / varianceA.get(0, 0);
    double weightAy = 1.0 / varianceA.get(1, 0);
    double weightBx = 1.0 / varianceB.get(0, 0);
    double weightBy = 1.0 / varianceB.get(1, 0);

    Pose2d fusedPose =
        new Pose2d(
            new Translation2d(
                (poseA.getTranslation().getX() * weightAx
                        + poseB.getTranslation().getX() * weightBx)
                    / (weightAx + weightBx),
                (poseA.getTranslation().getY() * weightAy
                        + poseB.getTranslation().getY() * weightBy)
                    / (weightAy + weightBy)),
            fusedHeading);

    Matrix<N3, N1> fusedStdDev =
        VecBuilder.fill(
            Math.sqrt(1.0 / (weightAx + weightBx)),
            Math.sqrt(1.0 / (weightAy + weightBy)),
            Math.sqrt(1.0 / (1.0 / varianceA.get(2, 0) + 1.0 / varianceB.get(2, 0))));

    int numTags = a.numTags() + b.numTags();
    double time = b.timestampSeconds();

    return new VisionEstimate(new Pose3d(fusedPose), time, fusedStdDev, numTags);
  }

  /** Helper to integrate a single estimate */
  private void integrateSingleEstimate(VisionEstimate estimate) {
    if (estimate != null) {
      drivetrain.addVisionMeasurement(
          estimate.pose().toPose2d(), estimate.timestampSeconds(), estimate.stdDevs());
    }
  }

  // ----- Simulation
  private void setupSimulation() {
    // Create the vision system simulation which handles cameras and targets on the field.
    visionSim = new VisionSystemSim("main");
    // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
    visionSim.addAprilTags(APRIL_TAG_FIELD_LAYOUT);
    startSimThread();
  }

  private void addCameraToSim(PhotonCamera camera, Transform3d cameraTransform) {
    // Create simulated camera properties. These can be set to mimic your actual camera.
    SimCameraProperties cameraProp = new SimCameraProperties();
    cameraProp.setCalibration(1280, 800, Rotation2d.fromDegrees(70));
    cameraProp.setCalibError(0.35, 0.10);
    cameraProp.setFPS(50);
    cameraProp.setAvgLatencyMs(15);
    cameraProp.setLatencyStdDevMs(15);

    // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
    // targets.
    cameraSim = new PhotonCameraSim(camera, cameraProp);
    // Add the simulated camera to view the targets on this simulated field.
    visionSim.addCamera(cameraSim, cameraTransform);

    // Enable the raw and processed streams. These are enabled by default.
    // cameraSim.enableRawStream(false);
    // cameraSim.enableProcessedStream(false);

    // Enable drawing a wireframe visualization of the field to the camera streams.
    // This is extremely resource-intensive and is disabled by default.
    cameraSim.enableDrawWireframe(true);
  }

  private void startSimThread() {
    /* Run simulation at a faster rate so PID gains behave more reasonably */
    simNotifier =
        new Notifier(
            () -> {
              // Update camera simulation
              simulationPeriodic(drivetrain.getState().Pose);
              var debugField = getSimDebugField();
              debugField.getObject("EstimatedRobot").setPose(drivetrain.getState().Pose);
              // debugField.getObject("EstimatedRobotModules").setPoses(getState().Speeds);
            });
    simNotifier.startPeriodic(simLoopPeriod);
  }

  public void simulationPeriodic(Pose2d robotSimPose) {
    visionSim.update(robotSimPose);
  }

  /** Reset pose history of the robot in the vision system simulation. */
  public void resetSimPose(Pose2d pose) {
    if (Robot.isSimulation()) visionSim.resetRobotPose(pose);
  }

  /** A Field2d for visualizing our robot and objects on the field. */
  public Field2d getSimDebugField() {
    if (!Robot.isSimulation()) return null;
    return visionSim.getDebugField();
  }
}
