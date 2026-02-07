package frc.robot.subsystems.Turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.constants.VisionConstants.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class TurretController extends SubsystemBase {

  private static final String TURRET_CAMERA = "TurretCamera";

  PhotonCamera camera = new PhotonCamera(TURRET_CAMERA);

  public static final Transform3d TURRET_TRANSFORM =
      new Transform3d(
          new Translation3d(-.272, 0.172, 0.711),
          new Rotation3d(Degrees.zero(), Degrees.of(-18), Degrees.of(0)));

  public static final Transform3d TAG_10_TO_HUB_CENTER =
      new Transform3d(new Translation3d(.5842, 0.0, 0.0), new Rotation3d());

  static final Map<Integer, Transform3d> tagToHubCenterMap =
      Map.of(10, new Transform3d(new Translation3d(-.5842, 0.0, 0.0), new Rotation3d()));

  static final List<Integer> redHubTags = List.of(10); // , 11, 8, 9, 2, 5);
  static final List<Integer> blueHubTags = List.of(26, 27, 24, 25, 18, 21);

  static final Map<DriverStation.Alliance, List<Integer>> hubTagsMap =
      Map.of(
          DriverStation.Alliance.Red, redHubTags,
          DriverStation.Alliance.Blue, blueHubTags);

  private List<Integer> hubTags = new ArrayList<>();
  private Supplier<Pose2d> poseSupplier;

  private Consumer<Angle> turret;

  // Simulation
  private PhotonCameraSim cameraSim;

  private VisionSystemSim visionSim;

  public Transform3d cameraTransform =
      new Transform3d(
          new Translation3d(-.272, 0.172, 0.711),
          new Rotation3d(Degrees.zero(), Degrees.of(-18), Degrees.of(0)));

  public TurretController(Supplier<Pose2d> poseSupplier, Consumer<Angle> turret) {
    this.poseSupplier = poseSupplier;
    this.turret = turret;
    Consumer<DriverStation.Alliance> setHubTags =
        value -> hubTags = (value == DriverStation.Alliance.Red ? redHubTags : blueHubTags);
    DriverStation.getAlliance()
        .ifPresentOrElse(
            setHubTags,
            () -> {
              hubTags.addAll(redHubTags);
              hubTags.addAll(blueHubTags);
            });

    // ----- Simulation
    if (Robot.isSimulation()) {
      // Create the vision system simulation which handles cameras and targets on the field.
      visionSim = new VisionSystemSim("turret");
      // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
      visionSim.addAprilTags(RED_HUB_TAGS_ONLY_LAYOUT);
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

      startSimThread();
    }
  }

  @Override
  public void periodic() {
    processCameraInput(camera);
  }

  private double turretAngleRadians = 0.0;

  private double hubDistance = 0.0;

  public void updateCameraAngle() {
    if (Robot.isSimulation()) {
      visionSim.adjustCamera(
          cameraSim,
          new Transform3d(
              cameraTransform.getTranslation(),
              new Rotation3d(
                  cameraTransform.getRotation().getX(),
                  cameraTransform.getRotation().getY(),
                  turretAngleRadians)));
    }
    turret.accept(Radians.of(turretAngleRadians));
  }

  public void processCameraInput(PhotonCamera camera) {
    List<PhotonPipelineResult> cameraResults = camera.getAllUnreadResults();
    if (cameraResults.isEmpty()) return;

    PhotonPipelineResult cameraResult = cameraResults.get(cameraResults.size() - 1);

    List<PhotonTrackedTarget> targets = cameraResult.getTargets();
    Transform3d hubCenter = null;

    targetLoop:
    for (PhotonTrackedTarget target : targets) {
      // can we see a hub tag  .. 10, 11, 8, 9, 2, 5   or 26, 27, 24, 25, 18, 21
      for (Integer id : hubTags) {
        if (target.getFiducialId() == id) {
          hubCenter = target.getBestCameraToTarget().plus(tagToHubCenterMap.get(id));
          break targetLoop;
        }
      }
    }

    if (hubCenter != null) {
      hubDistance = hubCenter.getTranslation().getNorm();

      double radiansOffset = Math.atan(hubCenter.getY() / hubCenter.getX());

      turretAngleRadians = turretAngleRadians + radiansOffset;
      updateCameraAngle();
    }
  }

  @Logged
  public Translation2d getTargetedHubPosition() {
    Transform2d targetTransform =
        new Transform2d(new Translation2d(hubDistance, 0.0), Rotation2d.k180deg);

    Transform2d flattenedCameraTransform =
        new Transform2d(
            cameraTransform.getX(),
            cameraTransform.getY(),
            cameraTransform.getRotation().toRotation2d());

    Pose2d turretPose =
        poseSupplier
            .get()
            .transformBy(flattenedCameraTransform)
            .transformBy(new Transform2d(0, 0, new Rotation2d(turretAngleRadians)));

    Translation2d targetPos = turretPose.transformBy(targetTransform).getTranslation();
    return targetPos;
  }

  @Logged
  public Translation2d getPlus5TargetedHubPosition() {
    Transform2d targetTransform =
        new Transform2d(new Translation2d(hubDistance, 0.0), Rotation2d.k180deg);

    Transform2d flattenedCameraTransform =
        new Transform2d(
            cameraTransform.getX(),
            cameraTransform.getY(),
            cameraTransform.getRotation().toRotation2d());

    Angle plus5Degrees = Radians.of(turretAngleRadians).plus(Degrees.of(5));

    Pose2d turretPose =
        poseSupplier
            .get()
            .transformBy(flattenedCameraTransform)
            .transformBy(new Transform2d(0, 0, new Rotation2d(plus5Degrees)));

    Translation2d targetPos = turretPose.transformBy(targetTransform).getTranslation();
    return targetPos;
  }

  @Logged
  public Translation2d getMinus5TargetedHubPosition() {
    Transform2d targetTransform =
        new Transform2d(new Translation2d(hubDistance, 0.0), Rotation2d.k180deg);

    Transform2d flattenedCameraTransform =
        new Transform2d(
            cameraTransform.getX(),
            cameraTransform.getY(),
            cameraTransform.getRotation().toRotation2d());

    Angle minus5Degrees = Radians.of(turretAngleRadians).minus(Degrees.of(5));

    Pose2d turretPose =
        poseSupplier
            .get()
            .transformBy(flattenedCameraTransform)
            .transformBy(new Transform2d(0, 0, new Rotation2d(minus5Degrees)));

    Translation2d targetPos = turretPose.transformBy(targetTransform).getTranslation();
    return targetPos;
  }

  // ----- Simulation
  private static final double kSimLoopPeriod = 0.01;
  private Notifier m_simNotifier = null;

  private void startSimThread() {
    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier =
        new Notifier(
            () -> {
              // Update camera simulation
              simulationPeriodic(poseSupplier.get());

              var debugField = getSimDebugField();
              debugField.getObject("EstimatedRobot").setPose(poseSupplier.get());
            });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
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
