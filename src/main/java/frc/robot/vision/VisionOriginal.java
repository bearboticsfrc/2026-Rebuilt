package frc.robot.vision;

import static frc.robot.vision.VisionConstants.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Robot;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionOriginal {
  private final List<PhotonCamera> cameras = new ArrayList<>();
  private final List<PhotonPoseEstimator> photonEstimators = new ArrayList<>();

  public record VisionEstimate(EstimatedRobotPose pose, Matrix<N3, N1> stdDevs) {}
  ;

  private Matrix<N3, N1> curStdDevs;

  // Simulation
  private PhotonCameraSim cameraSim;
  private VisionSystemSim visionSim;

  @Logged(name = "Camera Poses", importance = Importance.CRITICAL)
  public Map<String, Pose2d> latestCameraPose = new HashMap<String, Pose2d>();

  @Logged(name = "Target Poses", importance = Importance.CRITICAL)
  public final List<Pose2d> targetPoses = new ArrayList<>();

  public VisionOriginal(List<VisionCamera> visionCameras) {
    for (VisionCamera visionCamera : visionCameras) {
      PhotonCamera camera = new PhotonCamera(visionCamera.getName());

      PhotonPoseEstimator photonEstimator =
          new PhotonPoseEstimator(APRIL_TAG_FIELD_LAYOUT, visionCamera.getTransform());

      cameras.add(camera);
      photonEstimators.add(photonEstimator);

      // ----- Simulation
      if (Robot.isSimulation()) {
        // Create the vision system simulation which handles cameras and targets on the field.
        visionSim = new VisionSystemSim("main");
        // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
        visionSim.addAprilTags(APRIL_TAG_FIELD_LAYOUT);
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
        visionSim.addCamera(cameraSim, visionCamera.getTransform());

        // Enable the raw and processed streams. These are enabled by default.
        // cameraSim.enableRawStream(false);
        // cameraSim.enableProcessedStream(false);

        // Enable drawing a wireframe visualization of the field to the camera streams.
        // This is extremely resource-intensive and is disabled by default.
        cameraSim.enableDrawWireframe(true);
      }
    }
  }

  /**
   * Returns the latest standard deviations of the estimated pose from {@link
   * #getEstimatedGlobalPose()}, This should only be used when there are targets visible.
   */
  public Matrix<N3, N1> getEstimationStdDevs() {
    return curStdDevs;
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
      Optional<EstimatedRobotPose> visionEstimation = Optional.empty();
      List<PhotonPipelineResult> changes = camera.getAllUnreadResults();
      if (changes.isEmpty()) continue;
      PhotonPipelineResult change = changes.get(changes.size() - 1);
      // for (PhotonPipelineResult change : changes) {
      // System.out.println(" Processing change for timestamp " + change.getTimestampSeconds());
      PhotonPoseEstimator photonPoseEstimator = photonEstimators.get(cameras.indexOf(camera));
      visionEstimation = photonPoseEstimator.estimateCoprocMultiTagPose(change);
      if (visionEstimation.isEmpty()) {
        visionEstimation = photonPoseEstimator.estimateLowestAmbiguityPose(change);
      }
      if (visionEstimation.isEmpty()) {
        continue;
      }
      if (isTooAmbiguous(visionEstimation.get()) || isTooFar(visionEstimation.get())) {
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

      // if (visionEstimation.isPresent()) {
      Matrix<N3, N1> stdDevs =
          calculateStdDevs(photonPoseEstimator, visionEstimation, change.getTargets());
      visionEstimates.add(new VisionEstimate(visionEstimation.get(), stdDevs));

      latestCameraPose.put(camera.getName(), visionEstimation.get().estimatedPose.toPose2d());
      // }
    }

    return visionEstimates;
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
    if (estimatedPose.isEmpty()) {
      // No pose input. Default to single-tag std devs
      return SINGLE_TAG_STD_DEVS;
    } else {
      // Pose present. Start running Heuristic
      var estStdDevs = SINGLE_TAG_STD_DEVS;
      int numTags = 0;
      double avgDist = 0;

      // Precalculation - see how many tags we found, and calculate an average-distance metric
      for (var tgt : targets) {
        var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
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
        // No tags visible. Default to single-tag std devs
        return SINGLE_TAG_STD_DEVS;
      } else {
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = MULTI_TAG_STD_DEVS;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
          return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else return estStdDevs.times(1 + (avgDist * avgDist / 30));
      }
    }
  }

  private void updatedTargetPoses(List<PhotonTrackedTarget> targetList) {
    for (PhotonTrackedTarget trackedTarget : targetList) {
      int fiducialId = trackedTarget.getFiducialId();
      Pose3d tagPose = VisionConstants.APRIL_TAG_FIELD_LAYOUT.getTagPose(fiducialId).get();

      targetPoses.add(tagPose.toPose2d());
    }
  }

  private boolean isTooFar(PhotonPipelineResult result) {
    return result.getBestTarget().bestCameraToTarget.getMeasureX().gt(CULLING_DISTANCE);
  }

  private boolean isTooFar(EstimatedRobotPose estimate) {
    return estimate.targetsUsed.get(0).getBestCameraToTarget().getMeasureX().gt(CULLING_DISTANCE);
  }

  private boolean isTooAmbiguous(PhotonPipelineResult result) {
    return result.getBestTarget().poseAmbiguity > CULLING_AMBIGUITY;
  }

  private boolean isTooAmbiguous(EstimatedRobotPose estimate) {
    return estimate.targetsUsed.get(0).poseAmbiguity > CULLING_AMBIGUITY;
  }

  // ----- Simulation
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
