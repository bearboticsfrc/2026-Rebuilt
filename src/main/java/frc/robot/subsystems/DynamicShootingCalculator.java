package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.RobotState;
import frc.robot.field.Field;
import lombok.Getter;

public class DynamicShootingCalculator {
  private static DynamicShootingCalculator instance;

  public static final Transform2d turretToRobot =
      new Transform2d(Inches.of(-6.25), Inches.of(-6.25), new Rotation2d(Degrees.of(180)));

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

  private static final InterpolatingDoubleTreeMap demoFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();

  private static final InterpolatingDoubleTreeMap demoHoodAngleMap =
      new InterpolatingDoubleTreeMap();

  private static final InterpolatingDoubleTreeMap demoTimeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  static {
    maxDistance = 12;
    minDistance = 1.17;

    /* Values for shooting */
    flywheelSpeedMap.put(1.17, 2400.0);
    flywheelSpeedMap.put(2.0, 2425.0);
    flywheelSpeedMap.put(2.67, 2625.0);
    flywheelSpeedMap.put(3.0, 2700.0);
    flywheelSpeedMap.put(3.57, 2900.0);
    flywheelSpeedMap.put(4.0, 3100.0);
    flywheelSpeedMap.put(4.67, 3250.0);
    flywheelSpeedMap.put(5.5, 3750.0);

    hoodAngleMap.put(1.17, 0.2);
    hoodAngleMap.put(2.0, 0.2);
    hoodAngleMap.put(2.67, 0.2);
    hoodAngleMap.put(3.0, 0.2);
    hoodAngleMap.put(3.57, 0.2);
    hoodAngleMap.put(4.0, 0.2);
    hoodAngleMap.put(4.67, 0.2);
    hoodAngleMap.put(5.5, 0.2);

    timeOfFlightMap.put(1.17, 0.94);
    timeOfFlightMap.put(2.0, 1.03);
    timeOfFlightMap.put(2.67, 1.1);
    timeOfFlightMap.put(3.0, 1.09);
    timeOfFlightMap.put(3.57, 1.3);
    timeOfFlightMap.put(4.0, 1.4);
    timeOfFlightMap.put(4.67, 1.46);
    timeOfFlightMap.put(5.5, 1.59);

    /* Values for passing */
    passFlywheelSpeedMap.put(4.7, 2000.0);
    passFlywheelSpeedMap.put(7.2, 2800.0);
    passFlywheelSpeedMap.put(8.7, 3200.0);
    passFlywheelSpeedMap.put(11.8, 4250.0);
    passFlywheelSpeedMap.put(14.5, 5400.0);

    passHoodAngleMap.put(4.7, 1.0);
    passHoodAngleMap.put(7.2, 1.0);
    passHoodAngleMap.put(8.7, 1.0);
    passHoodAngleMap.put(11.8, 1.0);
    passHoodAngleMap.put(14.5, 1.0);

    passTimeOfFlightMap.put(4.7, 1.0);
    passTimeOfFlightMap.put(14.5, 1.0);

    /* Demo Mode */
    // add values

    demoFlywheelSpeedMap.put(0.6, 1150.0);
    demoHoodAngleMap.put(0.6, 0.0);
    demoTimeOfFlightMap.put(1.0, 1.0);

    demoFlywheelSpeedMap.put(1.4, 1750.0);

    demoFlywheelSpeedMap.put(2.0, 2000.0);

    demoFlywheelSpeedMap.put(2.5, 2200.0);

    demoFlywheelSpeedMap.put(3.0, 2450.0);
    demoHoodAngleMap.put(3.0, 0.0);
    demoTimeOfFlightMap.put(3.0, 1.0);
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
    Pose2d estimatedPose = new Pose2d(8.08, 4.034536, new Rotation2d());

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

      double finalAngle =
          robotAngle
              + turretToRobot
                  .getRotation()
                  .getRadians(); // + fieldVel.omegaRadiansPerSecond * timeOfFlight;

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
        target.minus(lookaheadPose.getTranslation()).getAngle().minus(lookaheadPose.getRotation());
    lastTarget = target;

    // --- Lookup shot parameters ---
    double hoodAngle = getHoodMap().get(lookaheadDistance);
    double flywheelVelocity = getFlywheelMap().get(lookaheadDistance);
    boolean inRange = lookaheadDistance >= minDistance && lookaheadDistance <= maxDistance;

    latestParameters =
        new LaunchingParameters(inRange, turretAngle, turretVelocity, hoodAngle, flywheelVelocity);
    return latestParameters;
  }

  @Logged
  public Translation2d selectTarget() {
    RobotState state = RobotState.getInstance();
    if (state.isDemoMode() && state.isMoving()) return Field.getMyBackHub();
    if (state.isDemoMode()) return Field.getMyBackHub();
    // if (state.isInAllianceZone()) return Field.getMyHub();
    // if (state.isLeftNeutralZone()) return Field.getMyLeft();
    // if (state.isRightNeutralZone()) return Field.getMyRight();
    return Field.getMyHub();
  }

  private InterpolatingDoubleTreeMap getFlywheelMap() {
    RobotState state = RobotState.getInstance();
    // if (state.isDemoMode()) return demoFlywheelSpeedMap;
    // if (!state.isInAllianceZone()) return passFlywheelSpeedMap;
    return flywheelSpeedMap;
  }

  private InterpolatingDoubleTreeMap getHoodMap() {
    RobotState state = RobotState.getInstance();
    // if (state.isDemoMode()) return demoHoodAngleMap;
    // if (!state.isInAllianceZone()) return passHoodAngleMap;
    return hoodAngleMap;
  }

  private InterpolatingDoubleTreeMap getTOFMap() {
    // RobotState state = RobotState.getInstance();
    // if (state.isDemoMode()) return demoTimeOfFlightMap;
    // if (!state.isInAllianceZone()) return passTimeOfFlightMap;
    return timeOfFlightMap;
  }
}
