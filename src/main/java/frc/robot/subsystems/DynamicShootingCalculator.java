package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.RobotState;
import frc.robot.field.Field;

public class DynamicShootingCalculator{
  private static DynamicShootingCalculator instance;

  private Rotation2d lastTurretAngle;
  private double lastHoodAngle;
  private Rotation2d turretAngle;
  private double hoodAngle;
  private double turretVelocity;

  public static DynamicShootingCalculator getInstance() {
    if (instance == null) instance = new DynamicShootingCalculator();
    return instance;
  }

  private final LinearFilter turretAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / 0.02));
 
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

  private static final InterpolatingDoubleTreeMap hoodAngleMap =
      new InterpolatingDoubleTreeMap();

  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

static {

    maxDistance = 0.0;
    minDistance = 0.0;

    flywheelSpeedMap.put(2.55, 2500.0);
    flywheelSpeedMap.put(1.8, 2300.0);
    flywheelSpeedMap.put(3.5, 2700.0);
    flywheelSpeedMap.put(5.2, 3150.0);
    flywheelSpeedMap.put(4.0, 2800.0);
    flywheelSpeedMap.put(3.0, 2650.0);

    hoodAngleMap.put(2.55, 0.2);
    hoodAngleMap.put(1.8, 0.1);
    hoodAngleMap.put(3.5, 0.4);
    hoodAngleMap.put(5.2, 0.6);
    hoodAngleMap.put(4.0, 0.5);
    hoodAngleMap.put(3.0, 0.25);

    timeOfFlightMap.put(2.55, 0.0);
    timeOfFlightMap.put(1.8, 0.0);
    timeOfFlightMap.put(3.5, 0.0);
    timeOfFlightMap.put(5.2, 0.0);
    timeOfFlightMap.put(4.0, 0.0);
    timeOfFlightMap.put(3.0, 0.0);

  }

    public LaunchingParameters getParameters() {
    if (latestParameters != null) {
      return latestParameters;
    }

    //estimated robot pose
    Pose2d currentPose = RobotState.getInstance().robotPose;

    ChassisSpeeds robotRelativeVelocity = RobotState.getInstance().robotVelocity;

    //apply robot velocities to current pose
    Pose2d estimatedPose =
        currentPose.exp(
            new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond ,
                robotRelativeVelocity.vyMetersPerSecond,
                robotRelativeVelocity.omegaRadiansPerSecond));

    //get distance to Hub
    Pose2d turretPose = estimatedPose.transformBy(RobotState.getInstance().turretToRobot);

    double turretToHub = Field.getMyHub().getDistance(turretPose.getTranslation());

    //Calculate field relative turret velocity
    ChassisSpeeds robotVelocity = RobotState.getInstance().getFieldVelocity();

    double robotAngle = estimatedPose.getRotation().getRadians();

    double turretVelocityX =
        robotVelocity.vxMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (RobotState.getInstance().turretToRobot.getY() * Math.cos(robotAngle)
                    - RobotState.getInstance().turretToRobot.getX() * Math.sin(robotAngle));
    double turretVelocityY =
        robotVelocity.vyMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (RobotState.getInstance().turretToRobot.getX() * Math.cos(robotAngle)
                    - RobotState.getInstance().turretToRobot.getY() * Math.sin(robotAngle));

    // Account for imparted velocity by robot (turret) to offset
    double timeOfFlight;

    Pose2d lookaheadPose = turretPose;

    double lookaheadTurretToTargetDistance = turretToHub;

    for (int i = 0; i < 20; i++) {
      timeOfFlight = timeOfFlightMap.get(lookaheadTurretToTargetDistance);
      double offsetX = turretVelocityX * timeOfFlight;
      double offsetY = turretVelocityY * timeOfFlight;
      lookaheadPose =
          new Pose2d(
              turretPose.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              turretPose.getRotation());
      lookaheadTurretToTargetDistance = Field.getMyHub().getDistance(lookaheadPose.getTranslation());
    }

    // Calculate parameters accounted for imparted velocity
    turretAngle = Field.getMyHub().minus(lookaheadPose.getTranslation()).getAngle();
    hoodAngle = (hoodAngleMap.get(lookaheadTurretToTargetDistance));
    lastTurretAngle = turretAngle;
    lastHoodAngle = hoodAngle;
    turretVelocity =
        turretAngleFilter.calculate(
            turretAngle.minus(lastTurretAngle).getRadians() / 0.02);
    lastTurretAngle = turretAngle;
    lastHoodAngle = hoodAngle;
    latestParameters =
        new LaunchingParameters(
            lookaheadTurretToTargetDistance >= minDistance
                && lookaheadTurretToTargetDistance <= maxDistance,
            turretAngle,
            turretVelocity,
            hoodAngleMap.get(lookaheadTurretToTargetDistance),
            flywheelSpeedMap.get(lookaheadTurretToTargetDistance));

    return latestParameters;
  }

  public void clearLaunchingParameters() {
    latestParameters = null;
  }
}

