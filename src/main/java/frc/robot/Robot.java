// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import bearlib.fms.AllianceColor;
import bearlib.fms.AllianceReadyListener;
import bearlib.util.AllianceFlipUtil;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.rebuilt.HubTracker;
import frc.robot.rebuilt.Pilot;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.DriveTelemetry;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.intake.IntakeState;
import frc.robot.subsystems.intake.Rollers;
import frc.robot.subsystems.intake.Slider;
import frc.robot.subsystems.shooter.DynamicShootingCalculator;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.ShootState;
import frc.robot.subsystems.spindexer.Kicker;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.spindexer.SpindexerState;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretVisionHelper;
import frc.robot.subsystems.turret.TurretVisionHelper.TurretAimResult;
import frc.robot.test.SelfTest;
import frc.robot.vision.VisionConstants;
import frc.robot.vision.VisionSystem;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.Supplier;
import lombok.Getter;

public class Robot extends TimedRobot implements AllianceReadyListener {

  private static Robot instance = null;

  public static Robot get() {
    if (instance == null)
      throw new RuntimeException("Trying to access Robot static get before initialized.");
    return instance;
  }

  private final Importance MINIMUM_IMPORTANCE = Importance.DEBUG;

  private Command m_autonomousCommand;

  private final SendableChooser<Command> autoChooser;

  private final HubTracker tracker;

  @Logged private final Rollers rollers;

  @Logged private final Slider slider;

  @Logged private final Spindexer spindexer;

  @Logged private final Kicker kicker;

  @Logged @Getter private final Turret turret;

  @Logged @Getter public final VisionSystem vision;

  @Logged private final CommandSwerveDrivetrain drivetrain;

  @Logged private final Flywheel flywheel;

  @Logged private final Hood hood;

  @Logged private DynamicShootingCalculator calculator;

  @Logged private RobotState robotState = RobotState.getInstance();

  @Getter public Field2d field2d = new Field2d();

  private final TurretVisionHelper turretVisionHelper;

  @Logged private final IntakeState intakeState;

  @Logged private final ShootState shootState;

  @Logged private final SpindexerState spindexerState;

  private Command introspectedAutoCommand;

  @Logged(name = "Auto Start Pose", importance = Importance.CRITICAL)
  private Pose2d autoStartPose;

  private double MaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed

  private double MaxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private final DriveTelemetry driveTelemetry = new DriveTelemetry();

  private final SelfTest selfTest;

  public Robot() {
    instance = this;

    tracker = new HubTracker();
    rollers = new Rollers();
    slider = new Slider();
    spindexer = new Spindexer();
    kicker = new Kicker();
    turret = new Turret();
    drivetrain = TunerConstants.createDrivetrain();

    flywheel = new Flywheel();
    hood = new Hood();

    calculator = DynamicShootingCalculator.getInstance();

    vision =
        new VisionSystem(
            Arrays.asList(
                VisionConstants.REAR_CAMERA,
                VisionConstants.LEFT_CAMERA,
                VisionConstants.RIGHT_CAMERA),
            false,
            drivetrain,
            () -> turret.getPositionDegrees(),
            () -> drivetrain.getState().Speeds.omegaRadiansPerSecond);

    turretVisionHelper = new TurretVisionHelper();

    System.out.println("All subsystems Initialized");

    selfTest = new SelfTest(rollers, flywheel, hood, spindexer, kicker, turret, slider, drivetrain);

    autoChooser = AutoBuilder.buildAutoChooser("2O");
    SmartDashboard.putData("Auto Mode", autoChooser);
    configureLogging();
    selfTest.bindTriggers();
    configureDefaultCommands();

    shootState = new ShootState(flywheel, hood, calculator);

    spindexerState = new SpindexerState(kicker, spindexer, shootState);

    intakeState = new IntakeState(slider, rollers);

    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    AllianceColor.addListener(this);

    DriverStation.silenceJoystickConnectionWarning(false);

    new Trigger(() -> RobotState.getInstance().isInAllianceZone())
        .onTrue(Commands.runOnce(() -> resetCorrection()))
        .onFalse(Commands.runOnce(() -> resetCorrection()));
  }

  public CommandSwerveDrivetrain getSwerve() {
    return drivetrain;
  }

  public HubTracker getTracker() {
    return tracker;
  }

  @Logged
  public double getDistanceToHub() {
    return RobotState.getInstance().getDistanceToHub();
  }

  public Supplier<Double> getMaxLinearVelocity() {
    double distanceToHub = robotState.getDistanceToHub();
    return () -> (robotState.isShooting()) ? 1.15 - ((distanceToHub / 5.5) * 0.5) : MaxSpeed - 0.5;
  }

  public Supplier<Double> getMaxAngularVelocity() {
    return () ->
        (robotState.isShooting())
            ? RotationsPerSecond.of(0.25).in(RadiansPerSecond)
            : MaxAngularRate;
  }

  public void configureLogging() {
    Shuffleboard.stopRecording();

    DataLogManager.start("", "", 0.1);
    DriverStation.startDataLog(DataLogManager.getLog());

    Epilogue.configure(config -> config.minimumImportance = this.MINIMUM_IMPORTANCE);

    Epilogue.bind(this);
  }

  @Override
  public void robotPeriodic() {
    DriverStation.getAlliance().ifPresent(AllianceColor::setAllianceColor);
    drivetrain.updatePoses();
    CommandScheduler.getInstance().run();
    DynamicShootingCalculator.getInstance().clearLaunchingParameters();
  }

  @Override
  public void robotInit() {}

  // set turret
  private Command getTurretCommand() {
    return turret
        .setAngle(
            () -> getTargetTurretAngleRads(), () -> calculator.getParameters().turretVelocity())
        .withName("TurretCommand");
  }

  @Override
  public void autonomousInit() {
    resetCorrection();
    vision.updateCameraSettings();
    turretVisionHelper.updateLimelightSettings();

    CommandScheduler.getInstance().schedule(slider.calibrateZero().andThen(slider.retract()));

    m_autonomousCommand = getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    vision.updateCameraSettings();
    resetCorrection();
    turretVisionHelper.updateLimelightSettings();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();

    turret.removeDefaultCommand();
  }

  @Override
  public void testExit() {
    configureDefaultCommands();
  }

  @Override
  public void disabledPeriodic() {
    Command selectedAutoCommand = autoChooser.getSelected();

    if (introspectedAutoCommand != selectedAutoCommand
        && selectedAutoCommand instanceof PathPlannerAuto) {
      autoStartPose =
          AllianceFlipUtil.apply(((PathPlannerAuto) selectedAutoCommand).getStartingPose());
      introspectedAutoCommand = selectedAutoCommand;
      drivetrain.resetPose(autoStartPose);
      vision.resetPose();
      System.out.println("Setting autostartpose!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    }
  }

  @Override
  public void disabledInit() {
    vision.updateCameraSettings();
  }

  @Logged
  public double getMatchTime() {
    return DriverStation.getMatchTime();
  }

  public void configureDefaultCommands() {

    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        Pilot.getLeftY()
                            * getMaxLinearVelocity()
                                .get()) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        Pilot.getLeftX()
                            * getMaxLinearVelocity().get()) // Drive left with negative X (left)
                    .withRotationalRate(
                        Pilot.getRightX()
                            * getMaxAngularVelocity()
                                .get()) // Drive counterclockwise with negative X (left)
            ));

    drivetrain.registerTelemetry(driveTelemetry::telemeterize);

    turret.setDefaultCommand(getTurretCommand());
  }

  public Pose2d getPoseToResetTo() {

    Pose2d resetPose = AllianceFlipUtil.apply(new Pose2d(3.5, 4, new Rotation2d()));
    if (!RobotState.getInstance().isInAllianceZone()) {
      resetPose = AllianceFlipUtil.apply(new Pose2d(8.3, 4, new Rotation2d()));
    }
    return AllianceFlipUtil.apply(resetPose);
  }

  private double correctionOffsetRads = 0.0;
  private static final double CORRECTION_ALPHA = 0.3;
  private static final double MAX_PLAUSIBLE_CORRECTION = Units.degreesToRadians(5.0);

  public Angle getTargetTurretAngleRads() {
    double turretAngleRadians = calculator.getParameters().turretAngle().getMeasure().in(Radians);
    Optional<TurretAimResult> visionResult = turretVisionHelper.getHubAimOffset();
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    nt.getEntry("hasTurretVisionResult").setBoolean(visionResult.isPresent());

    if (visionResult.isPresent()
        && visionResult.get().tagCount() >= 2
        && RobotState.getInstance().isInAllianceZone()) {
      double rawCorrection = visionResult.get().yawOffset();
      nt.getEntry("turretVisionOffset").setDouble(rawCorrection);

      nt.getEntry("usingCorrection")
          .setBoolean((Math.abs(rawCorrection) < MAX_PLAUSIBLE_CORRECTION));

      if (Math.abs(rawCorrection) < MAX_PLAUSIBLE_CORRECTION) {
        correctionOffsetRads =
            CORRECTION_ALPHA * rawCorrection + (1 - CORRECTION_ALPHA) * correctionOffsetRads;
      }
    }

    nt.getEntry("turretCorrection").setDouble(correctionOffsetRads);
    return Radians.of(turretAngleRadians + correctionOffsetRads);
  }

  public void resetCorrection() {
    correctionOffsetRads = 0.0;
  }

  public void setupSmartDashboardData() {
    SmartDashboard.putData("Field2d", field2d);
  }

  private boolean initialPoseSet = false;

  @Override
  public void updateAlliance(Alliance alliance) {
    if (!initialPoseSet) {
      Command firstAuto = autoChooser.getSelected();
      if (firstAuto instanceof PathPlannerAuto) {
        System.out.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!FirstAuto: " + firstAuto.getName());
        drivetrain.resetPose(
            AllianceFlipUtil.apply(((PathPlannerAuto) firstAuto).getStartingPose()));
        vision.resetPose();
        initialPoseSet = true;
      } else {
        System.out.println("Firstauto is not a PathPlannerAuto:" + firstAuto.getName());
      }
    }
  }
}
