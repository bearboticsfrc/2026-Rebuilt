package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.vision.VisionConstants.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.field.AllianceFlipUtil;
import frc.robot.field.Field;
import frc.robot.subsystems.turret.TurretVisionHelper.TurretAimResult;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;
import lombok.*;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class TurretController extends SubsystemBase {

  private static final String TURRET_CAMERA = "TurretOV9281";

  @Getter public Angle minRotations = Rotations.of(-.45);
  @Getter public Angle maxRotations = Rotations.of(.45);

  PhotonCamera camera = new PhotonCamera(TURRET_CAMERA);

  TurretVisionHelper turretVisionHelper = new TurretVisionHelper();

  public static final Transform3d TURRET_TRANSFORM =
      new Transform3d(
          new Translation3d(-.272, 0.172, 0.711),
          new Rotation3d(Degrees.zero(), Degrees.of(-18), Degrees.of(0)));

  @SuppressWarnings("null")
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

  static final List<Integer> redHubTags = List.of(10, 5, 2, 8, 9, 11);
  static final List<Integer> blueHubTags = List.of(26, 27, 24, 25, 18, 21);

  static final Map<DriverStation.Alliance, List<Integer>> hubTagsMap =
      Map.of(
          DriverStation.Alliance.Red, redHubTags,
          DriverStation.Alliance.Blue, blueHubTags);

  private List<Integer> hubTags = new ArrayList<>();
  private Supplier<Pose2d> poseSupplier;

  private Consumer<Angle> turret;
  private Supplier<Angle> turretAngle;

  // Simulation
  private PhotonCameraSim cameraSim;

  private VisionSystemSim visionSim;

  public Transform3d cameraTransform =
      new Transform3d(
          new Translation3d(-.05, 0.1, 0.6),
          new Rotation3d(Degrees.zero(), Degrees.of(-24), Degrees.of(0)));

  public TurretController(
      Supplier<Pose2d> poseSupplier, Consumer<Angle> turret, Supplier<Angle> turretAngle) {
    this.poseSupplier = poseSupplier;
    this.turret = turret;
    this.turretAngle = turretAngle;
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
    if (active) {
      turretAngleRadians = turretAngle.get().in(Radians);
      processCameraInput();
    }
  }

  // Rolling average of the correction offset
  private double correctionOffsetRads = 0.0;
  private static final double CORRECTION_ALPHA = 0.15; // low pass filter weight
  private static final double MAX_PLAUSIBLE_CORRECTION = Units.degreesToRadians(5.0);

  public double getTargetTurretAngleRads(Pose2d globalRobotPose) {
    double globalAzimuth = 0; // computeAzimuthFromPose(globalRobotPose);

    Optional<TurretAimResult> visionResult = turretVisionHelper.getHubAimOffset();

    if (visionResult.isPresent() && visionResult.get().tagCount() >= 2) {
      double visionAzimuth = visionResult.get().yawOffset();
      double rawCorrection = visionAzimuth - globalAzimuth;

      // Reject implausible corrections - likely a bad solve
      if (Math.abs(rawCorrection) < MAX_PLAUSIBLE_CORRECTION) {
        // Low pass filter the correction to smooth it
        correctionOffsetRads =
            CORRECTION_ALPHA * rawCorrection + (1 - CORRECTION_ALPHA) * correctionOffsetRads;
      }
      // If correction is implausible, keep previous correction value
    }

    return globalAzimuth + correctionOffsetRads;
  }

  public void resetCorrection() {
    correctionOffsetRads = 0.0;
  }

  public Command startTrackingCommand() {
    return Commands.startEnd(() -> setActive(true), () -> setActive(false), this);
  }

  @Getter @Setter private boolean active = false;

  @Logged public double turretAngleRadians = 0.0;

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

  public void processCameraInput() {

    // Optional<TurretAimResult> turretAimResult = turretVisionHelper.getHubAimSolution();

    // if (turretAimResult.isEmpty()) {
    //   return;
    // }

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

      turretAngleRadians =
          wrapDegreesToSoftLimits(Radians.of(turretAngleRadians + radiansOffset)).in(Radians);
      updateCameraAngle();
    }
  }

  // constantly gets the angle from the robot to the hub (turret rotation relative to hub)
  @Logged
  public Angle getTurretRelativeAngle() {
    return AllianceFlipUtil.apply(
            Field.getMyHub().minus(poseSupplier.get().getTranslation()).getAngle())
        .getMeasure()
        .minus(AllianceFlipUtil.apply(poseSupplier.get().getRotation()).getMeasure());
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

  private Angle wrapDegreesToSoftLimits(Angle targetAngle) {
    Angle currentAngle = Radians.of(turretAngleRadians);

    // Solve for integer n such that minRotations <= targetDegrees + 360*n <= maxRotations
    int nMin = (int) Math.ceil(minRotations.minus(targetAngle).in(Degrees) / 360.0);
    int nMax = (int) Math.floor(maxRotations.minus(targetAngle).in(Degrees) / 360.0);

    if (nMin <= nMax) {
      // At least one equivalent fits in soft limits.
      int nClosest = (int) Math.round((currentAngle.minus(targetAngle).in(Degrees)) / 360.0);
      int n =
          Math.max(nMin, Math.min(nClosest, nMax)); // clamp the closest candidate to allowed range
      return Degrees.of(targetAngle.in(Degrees) + n * 360.0);
    } else {
      // No equivalent fits in soft limits -> clamp to nearest soft limit endpoint.
      double toMin = Math.abs(currentAngle.minus(minRotations).in(Degrees));
      double toMax = Math.abs(currentAngle.minus(maxRotations).in(Degrees));
      return (toMin < toMax) ? minRotations : maxRotations;
    }
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
