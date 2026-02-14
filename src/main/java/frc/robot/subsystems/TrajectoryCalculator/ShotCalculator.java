package frc.robot.subsystems.TrajectoryCalculator;

import static edu.wpi.first.units.Units.Degrees;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.function.Supplier;

public class ShotCalculator {

  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;
  private final Translation2d blueHub;
  private final Translation2d blueOutpost;
  private final Translation2d blueDepot;
  
  public ShotCalculator(
      Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
    this.poseSupplier = poseSupplier;
    this.chassisSpeedsSupplier = chassisSpeedsSupplier;
    // Initialize alliance-dependent field positions here (safe to call DriverStation)
    if (DriverStation.getAlliance().map(a -> a == DriverStation.Alliance.Blue).orElse(false)) {
      blueHub = new Translation2d(4.63, 4.03);
      blueOutpost = new Translation2d(0.43, 0.3);
      blueDepot = new Translation2d(0.43, 7.5);
    } else {
      blueHub = new Translation2d(12.04, 4.03);
      blueOutpost = new Translation2d(16.24, 7.5);
      blueDepot = new Translation2d(16.24, 0.3);
    }
  }

  private Rotation2d getBotYaw() {
    return poseSupplier.get().getRotation();
  }

  private Translation2d getHub() {
    return FlippingUtil.flipFieldPosition(blueHub);
  }

  private Translation2d getOutpost() {
    return FlippingUtil.flipFieldPosition(blueOutpost);
  }

  private Translation2d getDepot() {
    return FlippingUtil.flipFieldPosition(blueDepot);
  }

  @Logged
  private double getHubDistance() {
    return ((poseSupplier.get().getTranslation()).getDistance(getHub()));
  }

  @Logged
  private double getOutpostDistance() {
    return ((poseSupplier.get().getTranslation()).getDistance(getOutpost()));
  }

  @Logged
  private double getDepotDistance() {
    return ((poseSupplier.get().getTranslation()).getDistance(getDepot()));
  }

  @Logged
  private double getHubDistance(Pose2d location) {
    return ((location.getTranslation()).getDistance(getHub()));
  }

  @Logged
  private double getOutpostDistance(Pose2d location) {
    return ((location.getTranslation()).getDistance(getOutpost()));
  }

  @Logged
  private double getDepotDistance(Pose2d location) {
    return ((location.getTranslation()).getDistance(getDepot()));
  }

  @Logged Rotation2d robotRotation;
  @Logged Rotation2d turretRotation;
  @Logged Angle turretRelativeRotation;
  @Logged Rotation2d futureRobotRotation;
  @Logged Rotation2d futureTurretRotation;
  @Logged Angle futureTurretRelativeRotation;
  @Logged Pose2d futureLocation = new Pose2d();

  private void updateFutureTurretRotation(Pose2d futurePose) {
    futureRobotRotation = futurePose.getRotation();
    futureTurretRotation = ((getHub().minus(futurePose.getTranslation())).getAngle());
    futureTurretRelativeRotation =
        Degrees.of(futureRobotRotation.minus(futureTurretRotation).getDegrees());
    this.futureLocation = new Pose2d(futurePose.getTranslation(), futureTurretRotation);
  }

  @Logged double botVx;
  @Logged double botVy;
  @Logged double turretVx;
  @Logged double turretVy;
  @Logged double dx;
  @Logged double dy;
  @Logged double dTheta;

  /**
   * Predicts the robot's pose after a given time interval for ANY drivetrain.
   *
   * @param currentPose Current robot pose (from odometry)
   * @param chassisSpeeds Current chassis speeds (vx, vy in m/s, omega in rad/s)
   * @param deltaTimeSeconds Time into the future to predict
   * @return Predicted future pose
   */
  private Pose2d predictFuturePose(
      Pose2d currentPose, ChassisSpeeds chassisSpeeds, double deltaTimeSeconds) {

    // Calculate change in position over deltaTime
    double turretAngularVelocity =
        chassisSpeeds.omegaRadiansPerSecond * 1.41061531; // finds angular speed
    Rotation2d botYaw = getBotYaw();

    botVx = chassisSpeeds.vxMetersPerSecond; // velocity of the robot in the y direction
    botVy = chassisSpeeds.vyMetersPerSecond; // velocity of the robot in the x direction
    turretVx = turretAngularVelocity * cos(botYaw.getRadians() - (3 / 4 * Math.PI));
    // finds velocity of the turret in the y direction
    turretVy = turretAngularVelocity * sin(botYaw.getRadians() - (3 / 4 * Math.PI));
    // finds velocity of the turret in the x direction
    dx = (botVx + turretVx) * deltaTimeSeconds;
    dy = (botVy + turretVy) * deltaTimeSeconds;

    dTheta = chassisSpeeds.omegaRadiansPerSecond * deltaTimeSeconds;

    // Create a transform representing the movement
    Transform2d transform = new Transform2d(dx, dy, new Rotation2d(dTheta));

    // Apply transform to current pose
    return currentPose.plus(transform);
  }

  // calculates trajectory, returns time, velocity, and angle of launch, 
  // will need to correct angle variance
  // Hood will need to be perpindicular to the returned angle
  private double[] getHubTrajectorySolutions() {
    return TargetingSolver.solveHubTrajectory(getHubDistance() * 3.280839895);
  }

  private double[] getDepotTrajectorySolutions() {
    return TargetingSolver.solveGroundTrajectory(getDepotDistance() * 3.280839895);
  }

  private double[] getOutpostTrajectorySolutions() {
    return TargetingSolver.solveGroundTrajectory(getOutpostDistance() * 3.280839895);
  }

  private double[] getHubTrajectorySolutions(Pose2d robotPose) {
    return TargetingSolver.solveHubTrajectory(getHubDistance(robotPose) * 3.280839895);
  }

  private double[] getDepotTrajectorySolutions(Pose2d robotPose) {
    return TargetingSolver.solveGroundTrajectory(getDepotDistance(robotPose) * 3.280839895);
  }

  private double[] getOutpostTrajectorySolutions(Pose2d robotPose) {
    return TargetingSolver.solveGroundTrajectory(getOutpostDistance(robotPose) * 3.280839895);
  }

  private ChassisSpeeds getChassisSpeeds() {
    return chassisSpeedsSupplier.get();
  }

  // calculates trajectory, returns time, speed(f/s), angle of launch, where turret needs to face,
  // will need to correct angle variance
  public double[] ShootOnMoveSolver(String targetLocation) {
    double[] trajectorySolution;
    if (targetLocation.equals("Outpost")) {
      trajectorySolution = getOutpostTrajectorySolutions();
      Pose2d futurePose =
          predictFuturePose(poseSupplier.get(), getChassisSpeeds(), trajectorySolution[0]);
      updateFutureTurretRotation(futurePose);
      trajectorySolution = getOutpostTrajectorySolutions(futurePose);
    } else if (targetLocation.equals("Depot")) {
      trajectorySolution = getDepotTrajectorySolutions();
      Pose2d futurePose =
          predictFuturePose(poseSupplier.get(), getChassisSpeeds(), trajectorySolution[0]);
      updateFutureTurretRotation(futurePose);
      trajectorySolution = getDepotTrajectorySolutions(futurePose);
    } else {
      trajectorySolution = getHubTrajectorySolutions();
      Pose2d futurePose =
          predictFuturePose(poseSupplier.get(), getChassisSpeeds(), trajectorySolution[0]);
      updateFutureTurretRotation(futurePose);
      trajectorySolution = getHubTrajectorySolutions(futurePose);
    }
    return new double[] {
      trajectorySolution[0],
      trajectorySolution[1],
      trajectorySolution[2],
      futureTurretRotation.getDegrees()
    };
  }

  public double flywheelRPMFromVelocity(double velocityFPS) {
    // Convert velocity from feet per second to meters per second
    double velocityMPS = velocityFPS * 0.3048;
    // Calculate the required flywheel RPM using the formula: RPM = (velocity * 60) / (2 * π * radius)
    // Flywheel diameter of 0.1007 meters (3.965 inches)
    double flywheelDiameter = 0.1007;
    return (velocityMPS * 60) / (Math.PI * flywheelDiameter);
  }
  
}
