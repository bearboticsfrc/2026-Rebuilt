package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotState;
import frc.robot.field.Field;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import java.util.function.Supplier;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily
 * be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
  private static final double kSimLoopPeriod = 0.005; // 5ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  private static final Matrix<N3, N1> STD_DEVS = VecBuilder.fill(0.1, 0.1, 0.05);

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean m_hasAppliedOperatorPerspective = false;

  /** Swerve request to apply during robot-centric path following */
  private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds =
      new SwerveRequest.ApplyRobotSpeeds();

  /* Swerve requests to apply during SysId characterization */
  private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization =
      new SwerveRequest.SysIdSwerveTranslation();

  // Buffer stores 1.5 seconds of pose history
  private final TimeInterpolatableBuffer<Pose2d> poseHistory =
      TimeInterpolatableBuffer.createBuffer(1.5);

  private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization =
      new SwerveRequest.SysIdSwerveSteerGains();
  private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization =
      new SwerveRequest.SysIdSwerveRotation();

  private final Timer motionTimer = new Timer();

  @Logged(name = "PigeonPitch")
  public double getPigeonPitch() {
    return getPigeon2().getPitch().getValueAsDouble();
  }

  @Logged(name = "PigeonRoll")
  public double getPigeonRoll() {
    return getPigeon2().getRoll().getValueAsDouble();
  }

  @Logged(name = "PigeonYaw")
  public double getPigeonYaw() {
    return getPigeon2().getYaw().getValueAsDouble();
  }

  @Logged(name = "IsOnBump")
  public boolean isOnBump() {
    return (Math.abs(getPigeonPitch()) >= 2.0 || Math.abs(getPigeonRoll()) >= 2);
  }

  public Pose2d getRobotPose() {
    return getState().Pose;
  }

  public Pose2d getPose() {
    return getState().Pose;
  }

  public ChassisSpeeds getCurrentRobotChassisSpeeds() {
    return getState().Speeds;
  }

  public ChassisSpeeds getChassisSpeeds() {
    return getState().Speeds;
  }

  public Pose2d getPoseAtTimestamp(double timestamp) {
    return poseHistory.getSample(timestamp).orElse(this.getState().Pose);
  }

  public boolean inMotion() {
    boolean spike = Math.abs(getPigeon2().getAccelerationX().getValueAsDouble()) > 0.05;

    if (spike) motionTimer.reset();

    return !motionTimer.hasElapsed(1.75);
  }

  /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
  private final SysIdRoutine m_sysIdRoutineTranslation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> setControl(m_translationCharacterization.withVolts(output)), null, this));

  /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
  private final SysIdRoutine m_sysIdRoutineSteer =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(7), // Use dynamic voltage of 7 V
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> setControl(m_steerCharacterization.withVolts(volts)), null, this));

  /*
   * SysId routine for characterizing rotation.
   * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
   * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
   */
  private final SysIdRoutine m_sysIdRoutineRotation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              /* This is in radians per second², but SysId only supports "volts per second" */
              Volts.of(Math.PI / 6).per(Second),
              /* This is in radians per second, but SysId only supports "volts" */
              Volts.of(Math.PI),
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
              },
              null,
              this));

  /* The SysId routine to test */
  private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

  private static final String[] MODULE_NAMES = {"FrontLeft", "FrontRight", "BackLeft", "BackRight"};
  private static final double STEER_TOLERANCE_DEG = 5.0;
  private static final double DRIVE_VELOCITY_TOLERANCE_RPS = 1.0;

  @Logged private boolean selfTestPassed = false;

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param modules Constants for each specific module
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
    configureAutoBuilder();
    setStateStdDevs(STD_DEVS);
    motionTimer.start();
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to
   *     0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
   * @param odometryStandardDeviation The standard deviation for odometry calculation in the form
   *     [x, y, theta]ᵀ, with units in meters and radians
   * @param visionStandardDeviation The standard deviation for vision calculation in the form [x, y,
   *     theta]ᵀ, with units in meters and radians
   * @param modules Constants for each specific module
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      Matrix<N3, N1> odometryStandardDeviation,
      Matrix<N3, N1> visionStandardDeviation,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(
        drivetrainConstants,
        odometryUpdateFrequency,
        odometryStandardDeviation,
        visionStandardDeviation,
        modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
    configureAutoBuilder();
    setStateStdDevs(STD_DEVS);
  }

  @Override
  public void resetPose(Pose2d newPose) {
    super.resetPose(newPose);
  }

  private void configureAutoBuilder() {
    try {
      var config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          () -> getState().Pose, // Supplier of current robot pose
          this::resetPose, // Consumer for seeding pose against auto
          () -> getState().Speeds, // Supplier of current robot speeds
          // Consumer of ChassisSpeeds and feedforwards to drive the robot
          (speeds, feedforwards) ->
              setControl(
                  m_pathApplyRobotSpeeds
                      .withSpeeds(speeds)
                      .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                      .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
          new PPHolonomicDriveController(
              // PID constants for translation
              new PIDConstants(10, 0, 0),
              // PID constants for rotation
              new PIDConstants(7, 0, 0)),
          config,
          // Assume the path needs to be flipped for Red vs Blue, this is normally the case
          () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
          this // Subsystem for requirements
          );
    } catch (Exception ex) {
      DriverStation.reportError(
          "Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
    }
  }

  /**
   * Returns a command that applies the specified control request to this swerve drivetrain.
   *
   * @param request Function returning the request to apply
   * @return Command to run
   */
  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  /**
   * Runs the SysId Quasistatic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Quasistatic test
   * @return Command to run
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.quasistatic(direction);
  }

  /**
   * Runs the SysId Dynamic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Dynamic test
   * @return Command to run
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.dynamic(direction);
  }

  @Override
  public void periodic() {
    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is disabled.
     * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
     */
    if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance()
          .ifPresent(
              allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
              });
    }
    checkForNAN();

    clampPoseToField();
  }

  /**
   * Self-test routine. Zeroes all steer motors, waits for them to settle, then checks: - Each steer
   * motor reached 0° within tolerance - Each drive motor responds to a brief slow command Results
   * are logged to SmartDashboard and the selfTestPassed field.
   */
  public Command selfTest() {
    final SwerveRequest.PointWheelsAt zeroWheels =
        new SwerveRequest.PointWheelsAt().withModuleDirection(Rotation2d.kZero);
    final SwerveRequest.RobotCentric driveSlowly =
        new SwerveRequest.RobotCentric().withVelocityX(0.3);

    return Commands.sequence(
            runOnce(
                () -> SmartDashboard.putString("Drivetrain/SelfTest/Status", "Testing steer...")),
            // Phase 1: zero all steer motors and wait for them to settle
            applyRequest(() -> zeroWheels).withTimeout(2.0),
            runOnce(this::checkSteerResults),
            runOnce(
                () -> SmartDashboard.putString("Drivetrain/SelfTest/Status", "Testing drive...")),
            // Phase 2: spin drive motors briefly and check velocity response
            applyRequest(() -> driveSlowly).withTimeout(0.75),
            runOnce(this::checkDriveResults))
        .finallyDo(
            () -> {
              setControl(new SwerveRequest.SwerveDriveBrake());
              SmartDashboard.putString(
                  "Drivetrain/SelfTest/Status", selfTestPassed ? "PASS" : "FAIL");
            })
        .withName("DrivetrainSelfTest");
  }

  private void checkSteerResults() {
    for (int i = 0; i < 4; i++) {
      double errorDeg = Math.abs(getState().ModuleStates[i].angle.getDegrees());
      boolean passed = errorDeg < STEER_TOLERANCE_DEG;
      SmartDashboard.putBoolean("Drivetrain/SelfTest/" + MODULE_NAMES[i] + "/SteerPassed", passed);
      SmartDashboard.putNumber(
          "Drivetrain/SelfTest/" + MODULE_NAMES[i] + "/SteerErrorDeg", errorDeg);
      if (!passed) selfTestPassed = false;
    }
  }

  private void checkDriveResults() {
    selfTestPassed = true;
    for (int i = 0; i < 4; i++) {
      double velocityRPS = Math.abs(getModule(i).getDriveMotor().getVelocity().getValueAsDouble());
      boolean passed = velocityRPS > DRIVE_VELOCITY_TOLERANCE_RPS;
      SmartDashboard.putBoolean("Drivetrain/SelfTest/" + MODULE_NAMES[i] + "/DrivePassed", passed);
      SmartDashboard.putNumber(
          "Drivetrain/SelfTest/" + MODULE_NAMES[i] + "/DriveVelocityRPS", velocityRPS);
      if (!passed) selfTestPassed = false;
    }
    SmartDashboard.putBoolean("Drivetrain/SelfTest/Passed", selfTestPassed);
  }

  public void updatePoses() {
    RobotState.getInstance().setRobotPose(getPose());
    RobotState.getInstance().setRobotVelocity(getState().Speeds);
    RobotState.getInstance().updatePose();
    poseHistory.addSample(Utils.getCurrentTimeSeconds(), this.getState().Pose);
  }

  private static final double HALF_ROBOT = Field.ROBOT_WIDTH / 2.0;

  private void checkForNAN() {
    Pose2d current = getStateCopy().Pose;
    if (current.getX() == Double.NaN || current.getY() == Double.NaN) {

      Pose2d lastPose = getPoseAtTimestamp(Utils.getCurrentTimeSeconds() - 0.02);
      System.out.println(
          ">>>>>>>>>>>>>> reseting pose to " + lastPose.getX() + ", " + lastPose.getY());
      resetPose(lastPose);
    }
  }

  private void clampPoseToField() {
    Pose2d current = getStateCopy().Pose;
    double clampedX = MathUtil.clamp(current.getX(), HALF_ROBOT, Field.LENGTH - HALF_ROBOT);
    double clampedY = MathUtil.clamp(current.getY(), HALF_ROBOT, Field.WIDTH - HALF_ROBOT);
    if (clampedX != current.getX() || clampedY != current.getY()) {
      resetPose(new Pose2d(new Translation2d(clampedX, clampedY), current.getRotation()));
    }
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - m_lastSimTime;
              m_lastSimTime = currentTime;

              /* use the measured time delta, get battery voltage from WPILib */
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
   * while still accounting for measurement noise.
   *
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds.
   */
  @Override
  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
  }

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
   * while still accounting for measurement noise.
   *
   * <p>Note that the vision measurement standard deviations passed into this method will continue
   * to apply to future measurements until a subsequent call to {@link
   * #setVisionMeasurementStdDevs(Matrix)} or this method.
   *
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds.
   * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement in the form
   *     [x, y, theta]ᵀ, with units in meters and radians.
   */
  @Override
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    super.addVisionMeasurement(
        visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
  }
}
