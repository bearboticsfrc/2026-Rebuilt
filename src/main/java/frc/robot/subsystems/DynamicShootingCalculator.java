package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.RobotState;
import frc.robot.field.Field;
import lombok.Getter;

public class DynamicShootingCalculator {
  private static DynamicShootingCalculator instance;

  public static final Transform2d turretToRobot =
      new Transform2d(Inches.of(-6.25), Inches.of(-6.25), new Rotation2d());

  private Translation2d lastTarget = Field.getMyHub();

  @Logged public Rotation2d turretAngle = new Rotation2d();

  @Logged @Getter public Pose2d lookaheadPose = new Pose2d();

  private AngularVelocity turretVelocity = RadiansPerSecond.of(0);

  private ChassisSpeeds filteredVel = new ChassisSpeeds();
  private static final double VELOCITY_FILTER_ALPHA = 0.1; // tune: lower = smoother

  public static DynamicShootingCalculator getInstance() {
    if (instance == null) instance = new DynamicShootingCalculator();
    return instance;
  }

  public record LaunchingParameters(
      boolean isValid,
      Rotation2d turretAngle,
      AngularVelocity turretVelocity,
      double hoodAngle,
      double flywheelVelocity) {}

  // Cache parameters
  private LaunchingParameters latestParameters = null;

  private static final double minDistance;

  private static final double maxDistance;

  private static final InterpolatingDoubleTreeMap flywheelSpeedMap =
      new InterpolatingDoubleTreeMap();

  private static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();

  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  private static final InterpolatingDoubleTreeMap passFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();

  private static final InterpolatingDoubleTreeMap passHoodAngleMap =
      new InterpolatingDoubleTreeMap();

  private static final InterpolatingDoubleTreeMap passTimeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  static {
    maxDistance = 10;
    minDistance = 1.17;

    /* Values for shooting */

    flywheelSpeedMap.put(1.17, 2400.0);
    flywheelSpeedMap.put(2.0, 2425.0);
    flywheelSpeedMap.put(2.67, 2625.0);
    flywheelSpeedMap.put(3.0, 2700.0);
    flywheelSpeedMap.put(3.57, 2900.0);
    flywheelSpeedMap.put(4.0, 3100.0);
    flywheelSpeedMap.put(4.67, 3250.0);
    flywheelSpeedMap.put(5.5, 3650.0);

    hoodAngleMap.put(1.17, 0.0);
    hoodAngleMap.put(2.0, 0.0);
    hoodAngleMap.put(2.67, 0.0);
    hoodAngleMap.put(3.0, 0.0);
    hoodAngleMap.put(3.57, 0.0);
    hoodAngleMap.put(4.0, 0.0);
    hoodAngleMap.put(4.67, 0.0);
    hoodAngleMap.put(5.5, 0.0);

    timeOfFlightMap.put(1.17, 0.94);
    timeOfFlightMap.put(2.0, 1.03);
    timeOfFlightMap.put(2.67, 1.1);
    timeOfFlightMap.put(3.0, 1.09);
    timeOfFlightMap.put(3.57, 1.3);
    timeOfFlightMap.put(4.0, 1.4);
    timeOfFlightMap.put(4.67, 1.46);
    timeOfFlightMap.put(5.5, 1.59);

    /* Values for passing */

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
    ChassisSpeeds rawVel = RobotState.getInstance().getFieldVelocity();

    if (RobotState.getInstance().isShooting()) {
      filteredVel =
          new ChassisSpeeds(
              filteredVel.vxMetersPerSecond
                  + VELOCITY_FILTER_ALPHA
                      * (rawVel.vxMetersPerSecond - filteredVel.vxMetersPerSecond),
              filteredVel.vyMetersPerSecond
                  + VELOCITY_FILTER_ALPHA
                      * (rawVel.vyMetersPerSecond - filteredVel.vyMetersPerSecond),
              rawVel.omegaRadiansPerSecond);
    } else {
      filteredVel = new ChassisSpeeds(0, 0, rawVel.omegaRadiansPerSecond);
    }
    ChassisSpeeds fieldVel = filteredVel;

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

    // --- Target selection ---
    Translation2d target = selectTarget();

    // --- Iterative lookahead: converge on self-consistent (distance, timeOfFlight) ---
    double lookaheadDistance = target.getDistance(turretPose.getTranslation());
    lookaheadPose = turretPose;

    final int MAX_ITERATIONS = 10;
    final double CONVERGENCE_THRESHOLD_M = 0.001;

    for (int i = 0; i < MAX_ITERATIONS; i++) {
      double timeOfFlight = getTOFMap().get(lookaheadDistance);

      // Propagate turret position and robot heading over timeOfFlight
      double propagatedAngle = robotAngle + fieldVel.omegaRadiansPerSecond * timeOfFlight / 2.0;

      double rx_avg =
          turretToRobot.getX() * Math.cos(propagatedAngle)
              - turretToRobot.getY() * Math.sin(propagatedAngle);
      double ry_avg =
          turretToRobot.getX() * Math.sin(propagatedAngle)
              + turretToRobot.getY() * Math.cos(propagatedAngle);
      double velX = fieldVel.vxMetersPerSecond - fieldVel.omegaRadiansPerSecond * ry_avg;
      double velY = fieldVel.vyMetersPerSecond + fieldVel.omegaRadiansPerSecond * rx_avg;

      double finalAngle = robotAngle; // + fieldVel.omegaRadiansPerSecond * timeOfFlight;

      lookaheadPose =
          new Pose2d(
              turretPose
                  .getTranslation()
                  .plus(new Translation2d(velX * timeOfFlight, velY * timeOfFlight)),
              new Rotation2d(finalAngle));

      double newDistance = target.getDistance(lookaheadPose.getTranslation());

      if (Math.abs(newDistance - lookaheadDistance) < CONVERGENCE_THRESHOLD_M) {
        lookaheadDistance = newDistance;

        break;
      }
      lookaheadDistance = newDistance;
    }
    // --- Turret angular velocity via finite difference, guarded against zone transitions ---
    boolean targetChanged = !target.equals(lastTarget);

    Translation2d turretToTargetVec = target.minus(lookaheadPose.getTranslation());
    double distanceToTarget = turretToTargetVec.getNorm();

    if (distanceToTarget > 0.1 && !targetChanged) {

      double ux = turretToTargetVec.getX() / distanceToTarget;
      double uy = turretToTargetVec.getY() / distanceToTarget;

      double finalPropagatedAngle =
          robotAngle + fieldVel.omegaRadiansPerSecond * getTOFMap().get(lookaheadDistance) / 2.0;

      double rx =
          turretToRobot.getX() * Math.cos(finalPropagatedAngle)
              - turretToRobot.getY() * Math.sin(finalPropagatedAngle);
      double ry =
          turretToRobot.getX() * Math.sin(finalPropagatedAngle)
              + turretToRobot.getY() * Math.cos(finalPropagatedAngle);
      double velX = fieldVel.vxMetersPerSecond - fieldVel.omegaRadiansPerSecond * ry;
      double velY = fieldVel.vyMetersPerSecond + fieldVel.omegaRadiansPerSecond * rx;
      double angleVelocity =
          (uy * velX - ux * velY) / distanceToTarget - fieldVel.omegaRadiansPerSecond;

      turretVelocity = RadiansPerSecond.of(angleVelocity);
    } else {
      turretVelocity = RadiansPerSecond.zero();
    }

    // --- Turret angle in robot frame at predicted pose ---

    turretAngle =
        target
            .minus(lookaheadPose.getTranslation())
            .getAngle()
            .minus(lookaheadPose.getRotation())
            .minus(turretToRobot.getRotation());

    lastTarget = target;

    // --- Lookup shot parameters ---
    double hoodAngle = getHoodMap().get(lookaheadDistance);
    double flywheelVelocity = getFlywheelMap().get(lookaheadDistance);
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

  private InterpolatingDoubleTreeMap getFlywheelMap() {
    RobotState state = RobotState.getInstance();
    return (!state.isInAllianceZone()) ? passFlywheelSpeedMap : flywheelSpeedMap;
  }

  private InterpolatingDoubleTreeMap getHoodMap() {
    RobotState state = RobotState.getInstance();
    return (!state.isInAllianceZone()) ? passHoodAngleMap : hoodAngleMap;
  }

  private InterpolatingDoubleTreeMap getTOFMap() {
    RobotState state = RobotState.getInstance();
    return (!state.isInAllianceZone()) ? passTimeOfFlightMap : timeOfFlightMap;
  }
}
