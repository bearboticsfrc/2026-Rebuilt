package frc.robot.subsystems.TrajectoryCalculator;

import java.util.ArrayList;
import java.util.List;

/** Represents a single point in the trajectory. */
class TrajectoryPoint {
  public double time;
  public double x;
  public double y;
  public double vx;
  public double vy;
  public double speed;

  public TrajectoryPoint(double time, double x, double y, double vx, double vy, double speed) {
    this.time = time;
    this.x = x;
    this.y = y;
    this.vx = vx;
    this.vy = vy;
    this.speed = speed;
  }
}

/**
 * Calculates the trajectory of a projectile with air resistance. Accounts for reduced air density
 * at 7000 feet elevation.
 */
public class ProjectileTrajectory {

  // Constants
  private static final double G = 32.174; // gravity in ft/s^2
  private static final double AIR_DENSITY_SEA_LEVEL = 0.002377; // slugs/ft^3 at sea level
  private static final double DENSITY_RATIO_AT_7000FT = 0.7374; // approximately 73.74% of sea level
  private static final double SLUG_TO_LBS = 32.174; // conversion factor
  private static final double SPHERE_DRAG_COEFFICIENT = 0.47; // drag coefficient for sphere

  private double height;
  private double speed;
  private double angleRad;
  private double dragCoefficient;
  private double crossSectionalArea;
  private double mass; // in slugs
  private double airDensity;
  private double vx0;
  private double vy0;
  private boolean airResistanceEnabled;
  private List<TrajectoryPoint> trajectory;

  /**
   * Initialize trajectory calculator.
   *
   * @param height Initial height in feet
   * @param speed Initial speed in ft/s
   * @param angleDegrees Launch angle in degrees (0-90)
   * @param dragCoefficient Dimensionless drag coefficient (default 0.47 for sphere)
   * @param crossSectionalArea Cross-sectional area in ft^2
   * @param massLbs Mass in pounds
   * @param airResistanceEnabled Whether to include air resistance in calculations
   */
  public ProjectileTrajectory(
      double height,
      double speed,
      double angleDegrees,
      double dragCoefficient,
      double crossSectionalArea,
      double massLbs,
      boolean airResistanceEnabled) {
    this.height = height;
    this.speed = speed;
    this.angleRad = Math.toRadians(angleDegrees);
    this.dragCoefficient = dragCoefficient;
    this.crossSectionalArea = crossSectionalArea;
    this.mass = massLbs / SLUG_TO_LBS; // Convert pounds to slugs
    this.airResistanceEnabled = airResistanceEnabled;

    // Air density at 7000 feet
    this.airDensity = AIR_DENSITY_SEA_LEVEL * DENSITY_RATIO_AT_7000FT;

    // Initial velocity components
    this.vx0 = speed * Math.cos(this.angleRad);
    this.vy0 = speed * Math.sin(this.angleRad);

    this.trajectory = new ArrayList<>();
  }

  /**
   * Initialize trajectory calculator with air resistance enabled by default.
   *
   * @param height Initial height in feet
   * @param speed Initial speed in ft/s
   * @param angleDegrees Launch angle in degrees (0-90)
   * @param dragCoefficient Dimensionless drag coefficient (default 0.47 for sphere)
   * @param crossSectionalArea Cross-sectional area in ft^2
   * @param massLbs Mass in pounds
   */
  public ProjectileTrajectory(
      double height,
      double speed,
      double angleDegrees,
      double dragCoefficient,
      double crossSectionalArea,
      double massLbs) {
    this(height, speed, angleDegrees, dragCoefficient, crossSectionalArea, massLbs, true);
  }

  /**
   * Initialize trajectory calculator with default drag parameters and air resistance enabled by
   * default.
   *
   * @param height Initial height in feet
   * @param speed Initial speed in ft/s
   * @param angleDegrees Launch angle in degrees (0-90)
   * @param massLbs Mass in pounds
   */
  public ProjectileTrajectory(double height, double speed, double angleDegrees, double massLbs) {
    this(height, speed, angleDegrees, 0.47, 0.01, massLbs, true);
  }

  /**
   * Calculate drag force using drag equation. F_drag = 0.5 * rho * v^2 * Cd * A Returns
   * deceleration due to drag in ft/s^2
   */
  private double calculateDragForce(double velocity) {
    if (velocity < 0.01) { // Avoid division issues at very low speeds
      return 0.0;
    }

    double dragForce =
        0.5 * airDensity * (velocity * velocity) * dragCoefficient * crossSectionalArea;
    return dragForce / mass; // Convert force to acceleration using object's mass
  }

  /**
   * Calculate the complete trajectory using numerical integration.
   *
   * @param maxTime Maximum simulation time in seconds
   * @param timeStep Time step for integration in seconds
   * @return List of TrajectoryPoint objects
   */
  public List<TrajectoryPoint> calculateTrajectory(double maxTime, double timeStep) {
    trajectory.clear();

    double t = 0.0;
    double x = 0.0;
    double y = height;
    double vx = vx0;
    double vy = vy0;

    while (t <= maxTime && y >= 0) {
      // Current speed
      double currentSpeed = Math.sqrt(vx * vx + vy * vy);

      // Store current point
      trajectory.add(new TrajectoryPoint(t, x, y, vx, vy, currentSpeed));

      // Calculate drag decelerations
      double axDrag = 0;
      double ayDrag = 0;

      if (airResistanceEnabled && currentSpeed > 0) {
        double dragDecel = calculateDragForce(currentSpeed);
        axDrag = -dragDecel * (vx / currentSpeed);
        ayDrag = -dragDecel * (vy / currentSpeed);
      }

      // Total accelerations (gravity + drag)
      double ax = axDrag;
      double ay = -G + ayDrag;

      // Update velocities (Euler method)
      vx += ax * timeStep;
      vy += ay * timeStep;

      // Update position
      x += vx * timeStep;
      y += vy * timeStep;

      t += timeStep;
    }

    // Add final point at ground level
    if (y < 0) {
      double finalSpeed = Math.sqrt(vx * vx + vy * vy);
      trajectory.add(new TrajectoryPoint(t, x, 0, vx, vy, finalSpeed));
    }

    return trajectory;
  }

  /** Calculate trajectory with default parameters (30 seconds, 0.01 second timestep). */
  public List<TrajectoryPoint> calculateTrajectory() {
    return calculateTrajectory(30, 0.01);
  }

  /** Get maximum height reached during trajectory. */
  public double getMaxHeight() {
    if (trajectory.isEmpty()) {
      return height;
    }
    double maxHeight = height;
    for (TrajectoryPoint point : trajectory) {
      if (point.y > maxHeight) {
        maxHeight = point.y;
      }
    }
    return maxHeight;
  }

  /** Get time when maximum height is reached. */
  public double getMaxHeightTime() {
    if (trajectory.isEmpty()) {
      return 0;
    }
    double maxHeight = height;
    double maxTime = 0;
    for (TrajectoryPoint point : trajectory) {
      if (point.y > maxHeight) {
        maxHeight = point.y;
        maxTime = point.time;
      }
    }
    return maxTime;
  }

  /** Get horizontal distance traveled (range). */
  public double getRange() {
    if (trajectory.isEmpty()) {
      return 0;
    }
    double maxRange = 0;
    for (TrajectoryPoint point : trajectory) {
      if (point.x > maxRange) {
        maxRange = point.x;
      }
    }
    return maxRange;
  }

  /** Get time when projectile hits ground. */
  public double getImpactTime() {
    if (trajectory.isEmpty()) {
      return 0;
    }
    // Find last point at or below ground
    for (int i = trajectory.size() - 1; i >= 0; i--) {
      TrajectoryPoint point = trajectory.get(i);
      if (point.y <= 0) {
        return point.time;
      }
    }
    return trajectory.get(trajectory.size() - 1).time;
  }

  /** Get speed at impact. */
  public double getImpactSpeed() {
    if (trajectory.isEmpty()) {
      return 0;
    }
    return trajectory.get(trajectory.size() - 1).speed;
  }

  /** Get velocity components at impact. */
  public double[] getImpactVelocity() {
    if (trajectory.isEmpty()) {
      return new double[] {0, 0};
    }
    TrajectoryPoint impactPoint = trajectory.get(trajectory.size() - 1);
    return new double[] {impactPoint.vx, impactPoint.vy};
  }

  /** Get trajectory points. */
  public List<TrajectoryPoint> getTrajectory() {
    return trajectory;
  }

  /**
   * Find all points where the projectile reaches a specific height. Returns a list of
   * TrajectoryPoints at that height.
   */
  public List<TrajectoryPoint> getPointsAtHeight(double targetHeight) {
    List<TrajectoryPoint> pointsAtHeight = new ArrayList<>();

    for (int i = 0; i < trajectory.size() - 1; i++) {
      TrajectoryPoint current = trajectory.get(i);
      TrajectoryPoint next = trajectory.get(i + 1);

      // Check if height crosses the target height between these points
      if ((current.y >= targetHeight && next.y <= targetHeight)
          || (current.y <= targetHeight && next.y >= targetHeight)) {

        // Interpolate to find more accurate point at target height
        double t = (targetHeight - current.y) / (next.y - current.y);
        double interpTime = current.time + t * (next.time - current.time);
        double interpX = current.x + t * (next.x - current.x);
        double interpVx = current.vx + t * (next.vx - current.vx);
        double interpVy = current.vy + t * (next.vy - current.vy);
        double interpSpeed = Math.sqrt(interpVx * interpVx + interpVy * interpVy);

        pointsAtHeight.add(
            new TrajectoryPoint(
                interpTime, interpX, targetHeight, interpVx, interpVy, interpSpeed));
      }
    }

    return pointsAtHeight;
  }

  /** Count how many times the projectile reaches a specific height. */
  public int countTimesAtHeight(double targetHeight) {
    return getPointsAtHeight(targetHeight).size();
  }

  /** Get height. */
  public double getHeight() {
    return height;
  }

  /** Get initial speed. */
  public double getSpeed() {
    return speed;
  }

  /** Get launch angle in degrees. */
  public double getAngleDegrees() {
    return Math.toDegrees(angleRad);
  }

  /** Get air density at current elevation. */
  public double getAirDensity() {
    return airDensity;
  }

  /** Get mass in pounds. */
  public double getMassPounds() {
    return mass * SLUG_TO_LBS;
  }

  /**
   * Create a sphere projectile with given diameter and mass.
   *
   * @param height Initial height in feet
   * @param speed Initial speed in ft/s
   * @param angleDegrees Launch angle in degrees
   * @param diameterInches Diameter of sphere in inches
   * @param massLbs Mass of sphere in pounds
   * @param airResistanceEnabled Whether to include air resistance
   * @return ProjectileTrajectory object
   */
  public static ProjectileTrajectory createSphere(
      double height,
      double speed,
      double angleDegrees,
      double diameterInches,
      double massLbs,
      boolean airResistanceEnabled) {
    // Convert diameter from inches to feet
    double diameterFeet = diameterInches / 12.0;

    // Calculate cross-sectional area (circle: π * r^2)
    double radius = diameterFeet / 2.0;
    double crossSectionalArea = Math.PI * radius * radius;

    return new ProjectileTrajectory(
        height,
        speed,
        angleDegrees,
        SPHERE_DRAG_COEFFICIENT,
        crossSectionalArea,
        massLbs,
        airResistanceEnabled);
  }

  /**
   * Factory method to create a sphere projectile with air resistance enabled by default.
   *
   * @param height Initial height in feet
   * @param speed Initial speed in ft/s
   * @param angleDegrees Launch angle in degrees
   * @param diameterInches Diameter of sphere in inches
   * @param massLbs Mass of sphere in pounds
   * @return ProjectileTrajectory object
   */
  public static ProjectileTrajectory createSphere(
      double height, double speed, double angleDegrees, double diameterInches, double massLbs) {
    return createSphere(height, speed, angleDegrees, diameterInches, massLbs, true);
  }
}
