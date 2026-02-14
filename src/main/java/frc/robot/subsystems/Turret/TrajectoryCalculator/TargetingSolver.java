package frc.robot.subsystems.Turret.TrajectoryCalculator;

/**
 * Trajectory targeting solver. Takes a target distance and finds the optimal launch velocity and
 * angle.
 */
public class TargetingSolver {

  private static final double SPHERE_DIAMETER_INCHES = 6;
  private static final double SPHERE_MASS_LBS = 0.5;
  private static final double DEFAULT_INITIAL_HEIGHT = 2; // feet

  private static final double MIN_VELOCITY = 10; // ft/s
  private static final double MAX_VELOCITY = 500; // ft/s

  private static final double DEFAULT_MIN_ANGLE = 35; // degrees
  private static final double DEFAULT_MAX_ANGLE = 80; // degrees

  private static final double ACCEPTABLE_ERROR = 0.25; // feet - convergence threshold
  private static final int MAX_ITERATIONS = 1000; // maximum iterations for optimization

  /** Represents a targeting solution. */
  static class TargetingSolution {
    double velocity;
    double angle;
    double achievedDistance;
    double achievedHeight; /* height at target distance (if constrained) */
    double distanceError;
    double heightError;
    double totalError;
    boolean heightConstrained;

    TargetingSolution(
        double velocity, double angle, double achievedDistance, double distanceError) {
      this.velocity = velocity;
      this.angle = angle;
      this.achievedDistance = achievedDistance;
      this.distanceError = distanceError;
      this.totalError = distanceError;
      this.heightConstrained = false;
    }

    TargetingSolution(
        double velocity,
        double angle,
        double achievedDistance,
        double achievedHeight,
        double distanceError,
        double heightError) {
      this.velocity = velocity;
      this.angle = angle;
      this.achievedDistance = achievedDistance;
      this.achievedHeight = achievedHeight;
      this.distanceError = distanceError;
      this.heightError = heightError;
      this.totalError = Math.sqrt(distanceError * distanceError + heightError * heightError);
      this.heightConstrained = true;
    }
  }

  public static double[] solveHubTrajectory(double distance) {
    return solveTrajectory(
        distance,
        6.0,
        0.5 /* potentially an issue that needs to be made positive instead of negative */,
        2.5,
        0.5,
        true);
  }

  public static double[] solveGroundTrajectory(double distance) {
    return solveTrajectory(
        distance,
        0,
        0.5 /* potentially an issue that needs to be made positive instead of negative */,
        -0.5,
        -0.5,
        false);
  }

  public static double[] solveTrajectory(
      double hubDistance,
      double heightAtTarget,
      Double rangeStartI,
      Double rangeEndI,
      double rangeHeightI,
      boolean airResistance) {
    double[] timeVelocityAngle = new double[3];

    double height = DEFAULT_INITIAL_HEIGHT;
    double targetDistance = hubDistance;

    // Get optional desired height at target distance(feet)
    Double desiredHeightAtTarget = heightAtTarget;

    // Automatic range constraint: 0.5 to 2.5 feet around target distance, must be above desired
    // height + 0.5 feet
    Double rangeStart = targetDistance + rangeStartI;
    Double rangeEnd = targetDistance + rangeEndI;
    double rangeHeight = desiredHeightAtTarget + rangeHeightI;
    String rangeDirection = "above";

    // Air resistance option
    boolean airResistanceEnabled = airResistance;

    // Get angle limits
    double minAngle = DEFAULT_MIN_ANGLE;
    double maxAngle = DEFAULT_MAX_ANGLE;

    // Find solution with constraints
    TargetingSolution solution =
        findOptimalTrajectory(
            height,
            targetDistance,
            desiredHeightAtTarget,
            rangeStart,
            rangeEnd,
            rangeDirection,
            rangeHeight,
            minAngle,
            maxAngle,
            airResistanceEnabled);

    // If no solution found or error too high with constraints, retry without range constraint
    if (solution == null || solution.distanceError > ACCEPTABLE_ERROR) {
      solution =
          findOptimalTrajectory(
              height,
              targetDistance,
              desiredHeightAtTarget,
              null,
              null,
              null,
              0.0,
              minAngle,
              maxAngle,
              airResistanceEnabled);
    }

    if (solution == null) {
      // Return NaNs to indicate failure
      timeVelocityAngle[0] = Double.NaN;
      timeVelocityAngle[1] = Double.NaN;
      timeVelocityAngle[2] = Double.NaN;
      return timeVelocityAngle;
    }

    // Build trajectory from the found solution to extract time-to-hit
    ProjectileTrajectory trajectory =
        ProjectileTrajectory.createSphere(
            height,
            solution.velocity,
            solution.angle,
            SPHERE_DIAMETER_INCHES,
            SPHERE_MASS_LBS,
            airResistanceEnabled);
    trajectory.calculateTrajectory();

    timeVelocityAngle[0] = getHeightHitInfo(trajectory, heightAtTarget); // time
    timeVelocityAngle[1] = getVelocity(solution); // velocity
    timeVelocityAngle[2] = getAngle(solution); // angle

    return timeVelocityAngle;
  }

  /**
   * Find optimal launch velocity and angle to hit target distance. Optionally also targets a
   * specific height at that distance. Optionally constrains projectile to be above/below height in
   * a distance range. Optionally constrains projectile not to exceed a ceiling height (lowest
   * priority). Uses iterative optimization to converge to within 0.5 feet of target.
   */
  private static TargetingSolution findOptimalTrajectory(
      double height,
      double targetDistance,
      Double desiredHeightAtTarget,
      Double rangeStart,
      Double rangeEnd,
      String rangeDirection,
      Double rangeHeight,
      double minAngle,
      double maxAngle,
      boolean airResistanceEnabled) {
    // Start with initial guess
    double currentVelocity = (MIN_VELOCITY + MAX_VELOCITY) / 2;
    double currentAngle = (minAngle + maxAngle) / 2;

    double velocityStep = (MAX_VELOCITY - MIN_VELOCITY) / 10;
    double angleStep = (maxAngle - minAngle) / 10;

    TargetingSolution bestSolution = null;
    double smallestError = Double.MAX_VALUE;
    int iterations = 0;

    while (iterations < MAX_ITERATIONS) {
      iterations++;

      // Evaluate current solution
      ProjectileTrajectory trajectory =
          ProjectileTrajectory.createSphere(
              height,
              currentVelocity,
              currentAngle,
              SPHERE_DIAMETER_INCHES,
              SPHERE_MASS_LBS,
              airResistanceEnabled);
      trajectory.calculateTrajectory();

      // Check range constraint if specified
      if (rangeStart != null && rangeEnd != null && rangeDirection != null) {
        boolean constraintMet =
            checkRangeConstraint(trajectory, rangeStart, rangeEnd, rangeHeight, rangeDirection);
        if (!constraintMet) {
          // Adjust angle to try to satisfy constraint
          currentAngle += (rangeDirection.equals("above") ? 1 : -1) * angleStep;
          currentAngle = Math.max(minAngle, Math.min(maxAngle, currentAngle));
          continue;
        }
      }

      double achievedDistance = trajectory.getRange();
      double distanceError = Math.abs(achievedDistance - targetDistance);

      double error;
      double achievedHeight = 0;

      if (desiredHeightAtTarget != null) {
        int timesHit = trajectory.countTimesAtHeight(desiredHeightAtTarget);
        if (timesHit >= 2) {
          // Get the SECOND occurrence of the desired height
          double distanceAtSecondHeight =
              getDistanceAtHeightOccurrence(trajectory, desiredHeightAtTarget, 2);
          if (distanceAtSecondHeight > 0) {
            // Focus only on horizontal distance error at the second hit
            achievedHeight = desiredHeightAtTarget;
            distanceError = Math.abs(distanceAtSecondHeight - targetDistance);
            error = distanceError; // Only care about horizontal distance
          } else {
            error = Double.MAX_VALUE; // Should not happen
          }
        } else {
          // Height not reached twice, very high error
          error = Double.MAX_VALUE;
        }
      } else {
        error = distanceError;
      }

      // Check for convergence
      if (error < smallestError) {
        smallestError = error;
        bestSolution =
            new TargetingSolution(
                currentVelocity, currentAngle, achievedDistance, achievedHeight, distanceError, 0);

        if (error <= ACCEPTABLE_ERROR && iterations >= 100) {
          return bestSolution;
        }
      }

      // Try adjusting velocity
      double velocityUp = currentVelocity + velocityStep;
      double velocityDown = currentVelocity - velocityStep;

      double errorUp =
          evaluateTrajectory(
              height,
              Math.min(MAX_VELOCITY, velocityUp),
              currentAngle,
              targetDistance,
              desiredHeightAtTarget,
              rangeStart,
              rangeEnd,
              rangeDirection,
              rangeHeight,
              airResistanceEnabled);
      double errorDown =
          evaluateTrajectory(
              height,
              Math.max(MIN_VELOCITY, velocityDown),
              currentAngle,
              targetDistance,
              desiredHeightAtTarget,
              rangeStart,
              rangeEnd,
              rangeDirection,
              rangeHeight,
              airResistanceEnabled);

      // Try adjusting angle
      double angleUp = currentAngle + angleStep;
      double angleDown = currentAngle - angleStep;

      double errorAngleUp =
          evaluateTrajectory(
              height,
              currentVelocity,
              Math.min(maxAngle, angleUp),
              targetDistance,
              desiredHeightAtTarget,
              rangeStart,
              rangeEnd,
              rangeDirection,
              rangeHeight,
              airResistanceEnabled);
      double errorAngleDown =
          evaluateTrajectory(
              height,
              currentVelocity,
              Math.max(minAngle, angleDown),
              targetDistance,
              desiredHeightAtTarget,
              rangeStart,
              rangeEnd,
              rangeDirection,
              rangeHeight,
              airResistanceEnabled);

      // Find the best adjustment
      double minError = error;
      int bestDirection = 0; // 0=none, 1=velUp, 2=velDown, 3=angleUp, 4=angleDown

      if (errorUp < minError) {
        minError = errorUp;
        bestDirection = 1;
      }
      if (errorDown < minError) {
        minError = errorDown;
        bestDirection = 2;
      }
      if (errorAngleUp < minError) {
        minError = errorAngleUp;
        bestDirection = 3;
      }
      if (errorAngleDown < minError) {
        minError = errorAngleDown;
        bestDirection = 4;
      }

      // Apply best adjustment
      switch (bestDirection) {
        case 0:
          // No improvement found, reduce step sizes
          velocityStep *= 0.8;
          angleStep *= 0.8;

          if (velocityStep < 0.1 && angleStep < 0.01) {
            // Steps are too small, converged
            return bestSolution;
          }
          break;
        case 1:
          currentVelocity = Math.min(MAX_VELOCITY, currentVelocity + velocityStep);
          break;
        case 2:
          currentVelocity = Math.max(MIN_VELOCITY, currentVelocity - velocityStep);
          break;
        case 3:
          currentAngle = Math.min(maxAngle, currentAngle + angleStep);
          break;
        case 4:
          currentAngle = Math.max(minAngle, currentAngle - angleStep);
          break;
      }
    }

    return bestSolution;
  }

  /** Evaluate the error for a given velocity and angle configuration. */
  private static double evaluateTrajectory(
      double height,
      double velocity,
      double angle,
      double targetDistance,
      Double desiredHeightAtTarget,
      Double rangeStart,
      Double rangeEnd,
      String rangeDirection,
      Double rangeHeight,
      boolean airResistanceEnabled) {
    ProjectileTrajectory trajectory =
        ProjectileTrajectory.createSphere(
            height, velocity, angle, SPHERE_DIAMETER_INCHES, SPHERE_MASS_LBS, airResistanceEnabled);
    trajectory.calculateTrajectory();

    // Check range constraint
    if (rangeStart != null && rangeEnd != null && rangeDirection != null) {
      if (!checkRangeConstraint(trajectory, rangeStart, rangeEnd, rangeHeight, rangeDirection)) {
        return Double.MAX_VALUE; // Invalid configuration
      }
    }

    double achievedDistance = trajectory.getRange();
    double distanceError = Math.abs(achievedDistance - targetDistance);

    if (desiredHeightAtTarget != null) {
      int timesHit = trajectory.countTimesAtHeight(desiredHeightAtTarget);

      if (timesHit >= 2) {
        double distanceAtSecondHeight =
            getDistanceAtHeightOccurrence(trajectory, desiredHeightAtTarget, 2);
        if (distanceAtSecondHeight > 0) {
          distanceError = Math.abs(distanceAtSecondHeight - targetDistance);
          return distanceError;
        } else {
          return Double.MAX_VALUE; // Should not happen
        }
      } else {
        return Double.MAX_VALUE; // Height not reached twice
      }
    }
    return distanceError;
  }

  /** Check if a trajectory maintains the height constraint in the given distance range. */
  private static boolean checkRangeConstraint(
      ProjectileTrajectory trajectory,
      double rangeStart,
      double rangeEnd,
      double constraintHeight,
      String direction) {
    java.util.List<TrajectoryPoint> points = trajectory.getTrajectory();

    if (points.isEmpty()) {
      return false;
    }

    // Check all points within the distance range
    for (TrajectoryPoint point : points) {
      if (point.x >= rangeStart && point.x <= rangeEnd) {
        if (direction.equals("above")) {
          // Projectile must be ABOVE the constraint height
          if (point.y < constraintHeight) {
            return false;
          }
        } else if (direction.equals("below")) {
          // Projectile must be BELOW the constraint height
          if (point.y > constraintHeight) {
            return false;
          }
        }
      }
    }

    return true;
  }

  /**
   * Find the distance at which the projectile reaches a specific height for the Nth occurrence.
   * Returns -1 if the height is not reached that many times.
   */
  private static double getDistanceAtHeightOccurrence(
      ProjectileTrajectory trajectory, double targetHeight, int occurrence) {
    java.util.List<TrajectoryPoint> pointsAtHeight = trajectory.getPointsAtHeight(targetHeight);

    if (occurrence > pointsAtHeight.size() || occurrence < 1) {
      return -1; // Height not reached that many times
    }

    // Return the distance at the requested occurrence (1-indexed)
    return pointsAtHeight.get(occurrence - 1).x;
  }

  /** return how long it took for fuel to get to the desired location */
  private static double getHeightHitInfo(ProjectileTrajectory trajectory, double targetHeight) {
    java.util.List<TrajectoryPoint> hitPoints = trajectory.getPointsAtHeight(targetHeight);

    if (hitPoints.size() >= 2) {
      // Return the time of the second occurrence (descending into the target)
      return hitPoints.get(1).time;
    } else if (hitPoints.size() == 1) {
      return hitPoints.get(0).time;
    } else {
      return Double.NaN;
    }
  }

  /** return optimal velocity */
  private static double getVelocity(TargetingSolution solution) {
    return solution.velocity;
  }

  /** return optimal angle of launch perpindicular to hood angle */
  private static double getAngle(TargetingSolution solution) {
    return solution.angle;
  }
}
