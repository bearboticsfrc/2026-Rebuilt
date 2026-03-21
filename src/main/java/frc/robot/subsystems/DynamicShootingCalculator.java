package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.RobotState;
import frc.robot.field.Field;
import lombok.Getter;

public class DynamicShootingCalculator {
  private static DynamicShootingCalculator instance;

  private Rotation2d lastTurretAngle = new Rotation2d();
  private Translation2d lastTarget = Field.getMyHub();

  @Logged public Rotation2d turretAngle = new Rotation2d();

  @Logged @Getter public Pose2d lookaheadPose = new Pose2d();

  private double hoodAngle = 0;
  private double flywheelVelocity = 0;
  private double turretVelocity = 0;

  private static final Transform2d turretToRobot = RobotState.turretToRobot;

  public static DynamicShootingCalculator getInstance() {
    if (instance == null) instance = new DynamicShootingCalculator();
    return instance;
  }

  private final LinearFilter turretAngleFilter = LinearFilter.movingAverage((int) (0.1 / 0.02));

  public record LaunchingParameters(
      boolean isValid,
      Rotation2d turretAngle,
      double turretVelocity,
      double hoodAngle,
      double flywheelVelocity) {}

  // Cache parameters
  private LaunchingParameters latestParameters = null;

  private static double minDistance;

  private static double maxDistance;

  private static final InterpolatingDoubleTreeMap flywheelSpeedMap =
      new InterpolatingDoubleTreeMap();

  private static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();

  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  static {
    maxDistance = 9;
    minDistance = 1.25;

    // angles for shooting
    flywheelSpeedMap.put(2.55, 2500.0);
    flywheelSpeedMap.put(1.8, 2300.0);
    flywheelSpeedMap.put(3.5, 2700.0);
    flywheelSpeedMap.put(5.2, 3150.0);
    flywheelSpeedMap.put(4.0, 2800.0);
    flywheelSpeedMap.put(3.0, 2650.0);

    // angles for passing

    hoodAngleMap.put(2.55, 0.2);
    hoodAngleMap.put(1.8, 0.1);
    hoodAngleMap.put(3.5, 0.4);
    hoodAngleMap.put(5.2, 0.6);
    hoodAngleMap.put(4.0, 0.5);
    hoodAngleMap.put(3.0, 0.25);

    // speeds for passing

    // get better time of flights
    timeOfFlightMap.put(1.8, 0.95);
    timeOfFlightMap.put(4.0, 1.15);
  }

  public LaunchingParameters getParameters_old() {
    if (latestParameters != null) {
      return latestParameters;
    }

    // estimated robot pose
    Pose2d currentPose = RobotState.getInstance().robotPose;

    ChassisSpeeds robotRelativeVelocity = RobotState.getInstance().robotVelocity;

    double phaseDelay = 0.03;
    // apply robot velocities to current pose
    Pose2d estimatedPose =
        currentPose.exp(
            new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

    // get distance to Hub
    Translation2d target;

    if (RobotState.getInstance().isInAllianceZone()) target = Field.getMyHub();
    else if (RobotState.getInstance().isLeftNeutralZone()) target = Field.getMyLeft();
    else if (RobotState.getInstance().isRightNeutralZone()) target = Field.getMyRight();
    else target = Field.getMyHub();

    Pose2d turretPose = estimatedPose.transformBy(turretToRobot);

    double turretToTarget = target.getDistance(turretPose.getTranslation());

    // Calculate field relative turret velocity
    ChassisSpeeds robotVelocity = RobotState.getInstance().getFieldVelocity();

    double robotAngle = estimatedPose.getRotation().getRadians();

    double rx_field =
        turretToRobot.getX() * Math.cos(robotAngle) - turretToRobot.getY() * Math.sin(robotAngle);
    double ry_field =
        turretToRobot.getX() * Math.sin(robotAngle) + turretToRobot.getY() * Math.cos(robotAngle);

    double turretVelocityX =
        robotVelocity.vxMetersPerSecond - robotVelocity.omegaRadiansPerSecond * (ry_field);
    double turretVelocityY =
        robotVelocity.vyMetersPerSecond + robotVelocity.omegaRadiansPerSecond * (rx_field);

    // Account for imparted velocity by robot (turret) to offset
    double timeOfFlight;

    lookaheadPose = turretPose;

    double lookaheadTurretToTargetDistance = turretToTarget;

    for (int i = 0; i < 5; i++) {
      timeOfFlight = timeOfFlightMap.get(lookaheadTurretToTargetDistance);
      double offsetX = turretVelocityX * timeOfFlight;
      double offsetY = turretVelocityY * timeOfFlight;
      lookaheadPose =
          new Pose2d(
              turretPose.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              turretPose
                  .getRotation()
                  .plus(new Rotation2d(robotVelocity.omegaRadiansPerSecond * timeOfFlight)));
      lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
    }

    // Calculate parameters accounted for imparted velocity
    turretAngle =
        target.minus(lookaheadPose.getTranslation()).getAngle().minus(lookaheadPose.getRotation());

    hoodAngle = (hoodAngleMap.get(lookaheadTurretToTargetDistance));
    flywheelVelocity = flywheelSpeedMap.get(lookaheadTurretToTargetDistance);
    turretVelocity =
        turretAngleFilter.calculate(turretAngle.minus(lastTurretAngle).getRadians() / 0.02);
    lastTurretAngle = turretAngle;

    latestParameters =
        new LaunchingParameters(
            lookaheadTurretToTargetDistance >= minDistance
                && lookaheadTurretToTargetDistance <= maxDistance,
            turretAngle,
            turretVelocity,
            hoodAngle,
            flywheelVelocity);
    return latestParameters;
  }

  public void clearLaunchingParameters() {
    latestParameters = null;
  }

  public LaunchingParameters getParameters() {
    if (latestParameters != null) {
      return latestParameters;
    }

    // --- Pose estimation with latency compensation ---
    Pose2d currentPose = RobotState.getInstance().robotPose;
    ChassisSpeeds robotRelVel = RobotState.getInstance().robotVelocity;
    ChassisSpeeds fieldVel = RobotState.getInstance().getFieldVelocity();

    final double PHASE_DELAY_S = 0.03;
    Pose2d estimatedPose =
        currentPose.exp(
            new Twist2d(
                robotRelVel.vxMetersPerSecond * PHASE_DELAY_S,
                robotRelVel.vyMetersPerSecond * PHASE_DELAY_S,
                robotRelVel.omegaRadiansPerSecond * PHASE_DELAY_S));

    // --- Turret pose in field frame ---
    Pose2d turretPose = estimatedPose.transformBy(turretToRobot);

    // --- Field-frame turret velocity (robot translation + rotation contribution) ---
    double robotAngle = estimatedPose.getRotation().getRadians();
    double rx_field =
        turretToRobot.getX() * Math.cos(robotAngle) - turretToRobot.getY() * Math.sin(robotAngle);
    double ry_field =
        turretToRobot.getX() * Math.sin(robotAngle) + turretToRobot.getY() * Math.cos(robotAngle);

    double turretVelX = fieldVel.vxMetersPerSecond - fieldVel.omegaRadiansPerSecond * ry_field;
    double turretVelY = fieldVel.vyMetersPerSecond + fieldVel.omegaRadiansPerSecond * rx_field;

    // --- Target selection ---
    Translation2d target = selectTarget();

    // --- Iterative lookahead: converge on self-consistent (distance, timeOfFlight) ---
    double lookaheadDistance = target.getDistance(turretPose.getTranslation());
    lookaheadPose = turretPose;

    final int MAX_ITERATIONS = 5;
    final double CONVERGENCE_THRESHOLD_M = 0.001;

    for (int i = 0; i < MAX_ITERATIONS; i++) {
      double timeOfFlight = timeOfFlightMap.get(lookaheadDistance);

      // Propagate turret position and robot heading over timeOfFlight
      double propagatedAngle = robotAngle + fieldVel.omegaRadiansPerSecond * timeOfFlight;

      // Recompute field-frame turret velocity at propagated angle for better accuracy
      double rx_prop =
          turretToRobot.getX() * Math.cos(propagatedAngle)
              - turretToRobot.getY() * Math.sin(propagatedAngle);
      double ry_prop =
          turretToRobot.getX() * Math.sin(propagatedAngle)
              + turretToRobot.getY() * Math.cos(propagatedAngle);
      double velX = fieldVel.vxMetersPerSecond - fieldVel.omegaRadiansPerSecond * ry_prop;
      double velY = fieldVel.vyMetersPerSecond + fieldVel.omegaRadiansPerSecond * rx_prop;

      lookaheadPose =
          new Pose2d(
              turretPose
                  .getTranslation()
                  .plus(new Translation2d(velX * timeOfFlight, velY * timeOfFlight)),
              new Rotation2d(propagatedAngle));

      double newDistance = target.getDistance(lookaheadPose.getTranslation());

      if (Math.abs(newDistance - lookaheadDistance) < CONVERGENCE_THRESHOLD_M) {
        lookaheadDistance = newDistance;
        break;
      }
      lookaheadDistance = newDistance;
    }

    // --- Turret angle in robot frame at predicted pose ---
    Rotation2d turretAngle =
        target.minus(lookaheadPose.getTranslation()).getAngle().minus(lookaheadPose.getRotation());

    // --- Turret angular velocity via finite difference, guarded against zone transitions ---
    double turretAngleDeltaRad = turretAngle.minus(lastTurretAngle).getRadians();
    boolean targetChanged = !selectTarget().equals(lastTarget);
    double turretVelocity =
        targetChanged ? 0.0 : turretAngleFilter.calculate(turretAngleDeltaRad / 0.02);

    lastTurretAngle = turretAngle;
    lastTarget = target;

    // --- Lookup shot parameters ---
    double hoodAngle = hoodAngleMap.get(lookaheadDistance);
    double flywheelVelocity = flywheelSpeedMap.get(lookaheadDistance);
    boolean inRange = lookaheadDistance >= minDistance && lookaheadDistance <= maxDistance;

    latestParameters =
        new LaunchingParameters(inRange, turretAngle, turretVelocity, hoodAngle, flywheelVelocity);
    return latestParameters;
  }

  private Translation2d selectTarget() {
    RobotState state = RobotState.getInstance();
    if (state.isInAllianceZone()) return Field.getMyHub();
    if (state.isLeftNeutralZone()) return Field.getMyLeft();
    if (state.isRightNeutralZone()) return Field.getMyRight();
    return Field.getMyHub();
  }
}
