// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

// originally from https://github.com/Mechanical-Advantage/RobotCode2023

package frc.robot.commands;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.Supplier;

/**
 * Drives the robot in a straight line to a specified target pose using PID controllers.
 *
 * <p>This command utilizes {@link ProfiledPIDController}s for independent control of the X and Y
 * translation components and the heading controller built into the {@link SwerveRequest} for
 * rotational control. The robot moves towards the target position while rotating to the target
 * heading.
 *
 * <p><b>Requirements:</b>
 *
 * <ul>
 *   <li>{@link CommandSwerveDrivetrain} subsystem
 * </ul>
 *
 * <p><b>Finish Condition:</b>
 *
 * <ul>
 *   <li>The robot reaches the target pose (within the specified translation and heading tolerances)
 *       and remains there for the duration defined by {@link #IS_FINISHED_DEBOUNCE_TIME}.
 * </ul>
 *
 * <p><b>End Behavior:</b>
 *
 * <ul>
 *   <li>Stops the drivetrain by sending a {@link SwerveRequest.Idle}.
 * </ul>
 */
public class DriveToPoseCommand extends Command {
  /** Default Proportional gain for the translation PID controllers. */
  private static final double DEFAULT_TRANSLATION_P = 2.5;

  /** Default Integral gain for the translation PID controllers. */
  private static final double DEFAULT_TRANSLATION_I = 0.0;

  /** Default Derivative gain for the translation PID controllers. */
  private static final double DEFAULT_TRANSLATION_D = 0.0;

  /** Default Proportional gain for the heading PID controller (derived from SysId). */
  private static final double DEFAULT_HEADING_P =
      6.469; // Derived from SysID - SwerveRequest.SysIdSwerveRotation

  /** Default Integral gain for the heading PID controller. */
  private static final double DEFAULT_HEADING_I = 0.0;

  /** Default Derivative gain for the heading PID controller. */
  private static final double DEFAULT_HEADING_D = 0.0;

  /** Default acceptable error for the heading controller. */
  private static final Angle DEFAULT_HEADING_TOLERANCE = Degrees.of(0.25);

  /** Default acceptable error for the translation controllers. */
  private static final Distance DEFAULT_TRANSLATION_TOLERANCE = Centimeters.of(1);

  /** Default maximum linear velocity for the trapezoidal motion profile. */
  private static final LinearVelocity DEFAULT_MAX_LINEAR_VELOCITY = MetersPerSecond.of(4.49);

  /** Default maximum linear acceleration for the trapezoidal motion profile. */
  private static final LinearAcceleration DEFAULT_MAX_LINEAR_ACCELERATION =
      MetersPerSecondPerSecond.of(3.0);

  /** Default constraints for the translational trapezoidal motion profile. */
  private static final TrapezoidProfile.Constraints DEFAULT_TRANSLATION_CONSTRAINTS =
      new TrapezoidProfile.Constraints(
          DEFAULT_MAX_LINEAR_VELOCITY.in(MetersPerSecond),
          DEFAULT_MAX_LINEAR_ACCELERATION.in(MetersPerSecondPerSecond));

  /** Time the robot must be within tolerances before the command finishes. */
  private static final Time IS_FINISHED_DEBOUNCE_TIME = Seconds.of(0.2);

  private final CommandSwerveDrivetrain drivetrain;
  private final ProfiledPIDController xTranslationController;
  private final ProfiledPIDController yTranslationController;
  private final Debouncer isFinishedDebouncer;
  private final SwerveRequest.FieldCentricFacingAngle driveRequest;

  private Supplier<Pose2d> poseSupplier;
  private Pose2d targetPose; // The target pose fetched during initialization

  /**
   * Constructs a new {@code DriveToPoseCommand} that drives the robot towards the specified pose.
   * Uses default PID gains, constraints, tolerances, and debounce time. The target pose is provided
   * by a {@link Supplier}, allowing the target to be determined dynamically when the command is
   * scheduled.
   *
   * @param drivetrain The drivetrain subsystem required by this command.
   * @param poseSupplier A supplier that returns the target {@link Pose2d} to drive to.
   */
  public DriveToPoseCommand(CommandSwerveDrivetrain drivetrain, Supplier<Pose2d> poseSupplier) {
    this.drivetrain = drivetrain;
    this.poseSupplier = poseSupplier;

    // Initialize translation PID controllers with default values
    this.xTranslationController =
        new ProfiledPIDController(
            DEFAULT_TRANSLATION_P,
            DEFAULT_TRANSLATION_I,
            DEFAULT_TRANSLATION_D,
            DEFAULT_TRANSLATION_CONSTRAINTS);
    this.yTranslationController =
        new ProfiledPIDController(
            DEFAULT_TRANSLATION_P,
            DEFAULT_TRANSLATION_I,
            DEFAULT_TRANSLATION_D,
            DEFAULT_TRANSLATION_CONSTRAINTS);

    // Set default tolerances for translation controllers
    this.xTranslationController.setTolerance(DEFAULT_TRANSLATION_TOLERANCE.in(Meters));
    this.yTranslationController.setTolerance(DEFAULT_TRANSLATION_TOLERANCE.in(Meters));

    // Initialize the SwerveRequest for field-centric control facing a target angle
    this.driveRequest =
        new FieldCentricFacingAngle()
            .withForwardPerspective(
                ForwardPerspectiveValue.BlueAlliance); // Set perspective (adjust if needed)

    // Configure the built-in heading controller within the SwerveRequest
    this.driveRequest.HeadingController.setPID(
        DEFAULT_HEADING_P, DEFAULT_HEADING_I, DEFAULT_HEADING_D);
    // Set default tolerance using Radians (consistent with internal PID math)
    this.driveRequest.HeadingController.setTolerance(DEFAULT_HEADING_TOLERANCE.in(Radians));

    // Initialize the debouncer for the isFinished condition
    this.isFinishedDebouncer = new Debouncer(IS_FINISHED_DEBOUNCE_TIME.in(Seconds));

    // Declare subsystem requirements
    addRequirements(drivetrain);
  }

  /**
   * Called once when the command is initially scheduled.
   *
   * <p>Retrieves the target pose from the supplier, resets the PID controllers with the current
   * robot pose and the target pose, and resets the finish condition debouncer.
   */
  @Override
  public void initialize() {
    final Pose2d currentPose = drivetrain.getState().Pose;
    targetPose = poseSupplier.get(); // Fetch the target pose dynamically

    // Reset translation controllers with current position and set target goal
    xTranslationController.reset(currentPose.getX());
    xTranslationController.setGoal(targetPose.getX());

    yTranslationController.reset(currentPose.getY());
    yTranslationController.setGoal(targetPose.getY());

    // Reset the heading controller (within the SwerveRequest)
    // The current heading is handled internally by the SwerveRequest system
    driveRequest.HeadingController.reset();

    // Reset the debouncer, indicating the robot has not yet reached the target state
    isFinishedDebouncer.calculate(false);

    // Reset drivetrain debug flags
    // drivetrain.xTranslationAtSetpoint = false;
    // drivetrain.yTranslationAtSetpoint = false;
    // drivetrain.headingAtSetpoint = false;
  }

  /**
   * Called repeatedly when this Command is scheduled.
   *
   * <p>Calculates the required field-relative X and Y velocities using the {@link
   * ProfiledPIDController}s based on the current robot pose. It potentially modifies these
   * velocities based on the selected {@link TranslationStrategy} (e.g., prioritizing one axis
   * before allowing the other). It then constructs and sends a {@link SwerveRequest} to the
   * drivetrain, commanding it to move with the calculated velocities and face the target pose's
   * rotation.
   */
  @Override
  public void execute() {
    final Pose2d currentPose = drivetrain.getState().Pose;

    // Calculate field-relative X and Y velocity outputs using profiled PID controllers
    double xVelocityOutput = xTranslationController.calculate(currentPose.getX());
    double yVelocityOutput = yTranslationController.calculate(currentPose.getY());

    // Send the potentially modified velocities and target heading to the drivetrain
    // The heading controller within driveRequest calculates the necessary rotational velocity
    // based on the current heading and the target rotation.
    drivetrain.setControl(
        driveRequest
            .withVelocityX(xVelocityOutput) // Set field-relative X velocity
            .withVelocityY(yVelocityOutput) // Set field-relative Y velocity
            .withTargetDirection(targetPose.getRotation())); // Set target orientation
  }

  /**
   * Called repeatedly when this Command is scheduled to determine if the command is finished.
   *
   * @return {@code true} if both the translation and heading controllers are within their
   *     respective tolerances ({@code atGoal()} for translation, {@code atSetpoint()} for heading)
   *     and have remained so for the duration specified by the debouncer; {@code false} otherwise.
   */
  @Override
  public boolean isFinished() {
    // Check if the translation controllers have reached their goal (position and velocity=0)
    boolean translationReached = xTranslationController.atGoal() && yTranslationController.atGoal();

    // Check if the heading controller has reached its setpoint (angle)
    boolean rotationReached = driveRequest.HeadingController.atSetpoint();

    // Update drivetrain status flags (useful for logging/debugging)
    // Note: atSetpoint() checks position only, atGoal() checks position and velocity
    // drivetrain.xTranslationAtSetpoint = xTranslationController.atSetpoint();
    // drivetrain.yTranslationAtSetpoint = yTranslationController.atSetpoint();
    // drivetrain.headingAtSetpoint = rotationReached;

    // Use the debouncer to ensure the robot has settled at the target pose
    return isFinishedDebouncer.calculate(translationReached && rotationReached);
  }

  /**
   * Called once when the command ends or is interrupted.
   *
   * <p>Stops the drivetrain by sending an idle request.
   *
   * @param interrupted {@code true} if the command was interrupted, {@code false} if it finished
   *     normally.
   */
  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(new SwerveRequest.Idle());

    // drivetrain.xTranslationAtSetpoint = false;
    // drivetrain.yTranslationAtSetpoint = false;
    // drivetrain.headingAtSetpoint = false;
  }

  /**
   * Sets the PID constants for the translation (X and Y) controllers.
   *
   * @param p Proportional gain.
   * @param i Integral gain.
   * @param d Derivative gain.
   * @return This command instance for method chaining.
   */
  public DriveToPoseCommand withTranslationPID(double p, double i, double d) {
    xTranslationController.setPID(p, i, d);
    yTranslationController.setPID(p, i, d);
    return this;
  }

  /**
   * Sets the PID constants and I-Zone for the translation (X and Y) controllers. The I-Zone defines
   * the range around the setpoint where the integral term is active.
   *
   * @param p Proportional gain.
   * @param i Integral gain.
   * @param d Derivative gain.
   * @param iZone The range around the setpoint where the integral term accumulates.
   * @return This command instance for method chaining.
   */
  public DriveToPoseCommand withTranslationPID(double p, double i, double d, Distance iZone) {
    xTranslationController.setPID(p, i, d);
    xTranslationController.setIZone(iZone.in(Meters));

    yTranslationController.setPID(p, i, d);
    yTranslationController.setIZone(iZone.in(Meters));

    return this;
  }

  /**
   * Sets the PID constants for the heading controller.
   *
   * @param p Proportional gain.
   * @param i Integral gain.
   * @param d Derivative gain.
   * @return This command instance for method chaining.
   */
  public DriveToPoseCommand withHeadingPID(double p, double i, double d) {
    driveRequest.HeadingController.setPID(p, i, d);
    return this;
  }

  /**
   * Sets the motion profile constraints for the translation (X and Y) controllers.
   *
   * @param maxLinearVelocity Maximum linear velocity.
   * @param maxLinearAcceleration Maximum linear acceleration.
   * @return This command instance for method chaining.
   */
  public DriveToPoseCommand withTranslationConstraints(
      LinearVelocity maxLinearVelocity, LinearAcceleration maxLinearAcceleration) {
    final TrapezoidProfile.Constraints translationConstraints =
        new TrapezoidProfile.Constraints(
            maxLinearVelocity.in(MetersPerSecond),
            maxLinearAcceleration.in(MetersPerSecondPerSecond));

    xTranslationController.setConstraints(translationConstraints);
    yTranslationController.setConstraints(translationConstraints);

    return this;
  }

  /**
   * Sets the tolerance for the translation (X and Y) controllers.
   *
   * @param tolerance The acceptable position error.
   * @return This command instance for method chaining.
   */
  public DriveToPoseCommand withTranslationTolerance(Distance tolerance) {
    this.xTranslationController.setTolerance(tolerance.in(Meters));
    this.yTranslationController.setTolerance(tolerance.in(Meters));

    return this;
  }

  /**
   * Sets the tolerance for the heading controller.
   *
   * @param tolerance The acceptable angle error.
   * @return This command instance for method chaining.
   */
  public DriveToPoseCommand withHeadingTolerance(Angle tolerance) {
    this.driveRequest.HeadingController.setTolerance(tolerance.in(Radians));
    return this;
  }

  /**
   * Sets the duration the robot must be within tolerances before the command is considered
   * finished.
   *
   * @param duration The debounce time duration.
   * @return This command instance for method chaining.
   */
  public DriveToPoseCommand withDebounceDuration(Time duration) {
    this.isFinishedDebouncer.setDebounceTime(duration.in(Seconds));
    return this;
  }

  /**
   * Sets the target pose supplier that the robot will align to.
   *
   * @param poseSupplier A supplier for the target {@link Pose2d}.
   * @return This command instance for method chaining.
   */
  public DriveToPoseCommand withPoseSupplier(Supplier<Pose2d> poseSupplier) {
    this.poseSupplier = poseSupplier;
    return this;
  }
}
/*
 @Logged(importance = Importance.CRITICAL)
  public boolean xTranslationAtSetpoint;

  @Logged(importance = Importance.CRITICAL)
  public boolean yTranslationAtSetpoint;

  @Logged(importance = Importance.CRITICAL)
  public boolean headingAtSetpoint;
*/
