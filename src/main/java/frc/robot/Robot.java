// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import bearlib.fms.AllianceColor;
import bearlib.fms.AllianceReadyListener;
import bearlib.util.AllianceFlipUtil;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.auto.Auton;
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
import frc.robot.subsystems.shooter.FlywheelState;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.HoodState;
import frc.robot.subsystems.spindexer.Kicker;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.spindexer.SpindexerState;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretState;
import frc.robot.test.SelfTest;
import frc.robot.vision.VisionConstants;
import frc.robot.vision.VisionSystem;
import java.util.Arrays;
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

  @Logged private final IntakeState intakeState;

  @Logged private final FlywheelState flywheelState;

  @Logged private final SpindexerState spindexerState;

  @Logged private final HoodState hoodState;

  @Logged private final TurretState turretState;

  private final Auton auton;

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

    System.out.println("All subsystems Initialized");

    selfTest = new SelfTest(rollers, flywheel, hood, spindexer, kicker, turret, slider, drivetrain);

    configureLogging();
    selfTest.bindTriggers();
    configureDefaultCommands();

    flywheelState = new FlywheelState(flywheel);

    spindexerState = new SpindexerState(kicker, spindexer, flywheelState);

    intakeState = new IntakeState(slider, rollers);

    hoodState = new HoodState(hood);

    turretState = new TurretState(turret);

    auton = new Auton(flywheelState, spindexerState, intakeState);

    AllianceColor.addListener(this);

    DriverStation.silenceJoystickConnectionWarning(false);
  }

  //
  // INITIALIZATION AND CONFIG.
  //

  @Override
  public void robotInit() {}

  @Override
  public void disabledInit() {}

  @Override
  public void teleopInit() {

    if (auton.getAutonomousCommand() != null) {
      auton.getAutonomousCommand().cancel();
    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();

    turret.removeDefaultCommand();
  }

  @Override
  public void autonomousInit() {
    CommandScheduler.getInstance().schedule(slider.calibrateZero().andThen(slider.retract()));

    if (auton.getAutonomousCommand() != null) {
      CommandScheduler.getInstance().schedule(auton.getAutonomousCommand());
    }
  }

  public void setupSmartDashboardData() {
    SmartDashboard.putData("Field2d", field2d);
  }

  public void configureLogging() {
    Shuffleboard.stopRecording();

    DataLogManager.start("", "", 0.1);
    DriverStation.startDataLog(DataLogManager.getLog());

    Epilogue.configure(config -> config.minimumImportance = this.MINIMUM_IMPORTANCE);

    Epilogue.bind(this);
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
  }

  private boolean initialPoseSet = false;

  @Override
  public void updateAlliance(Alliance alliance) {
    if (!initialPoseSet) {
      Command firstAuto = auton.getAutoChooser().getSelected();
      if (firstAuto instanceof PathPlannerAuto) {
        System.out.println("FirstAuto: " + firstAuto.getName());
        drivetrain.resetPose(
            AllianceFlipUtil.apply(((PathPlannerAuto) firstAuto).getStartingPose()));
        initialPoseSet = true;
      } else {
        System.out.println("Firstauto is not a PathPlannerAuto:" + firstAuto.getName());
      }
    }
  }

  @Override
  public void testExit() {
    configureDefaultCommands();
  }

  //
  // PERIODIC
  //

  @Override
  public void robotPeriodic() {
    DriverStation.getAlliance().ifPresent(AllianceColor::setAllianceColor);
    drivetrain.updatePoses();
    CommandScheduler.getInstance().run();
    DynamicShootingCalculator.getInstance().clearLaunchingParameters();
  }

  @Override
  public void disabledPeriodic() {
    Command selectedAutoCommand = auton.getAutoChooser().getSelected();

    if (introspectedAutoCommand != selectedAutoCommand
        && selectedAutoCommand instanceof PathPlannerAuto) {
      autoStartPose =
          AllianceFlipUtil.apply(((PathPlannerAuto) selectedAutoCommand).getStartingPose());
      introspectedAutoCommand = selectedAutoCommand;
      drivetrain.resetPose(autoStartPose);
      System.out.println("Setting autostartpose.........................");
    }
  }

  //
  // DRIVE
  //

  public CommandSwerveDrivetrain getSwerve() {
    return drivetrain;
  }

  /** Limit linear velocity in reference to distance from the hub. */
  public Supplier<Double> getMaxLinearVelocity() {
    return () -> (robotState.isShooting()) ? 0.5 : MaxSpeed - 0.5;
  }

  /** Limit Angular velocity. */
  public Supplier<Double> getMaxAngularVelocity() {
    return () ->
        (robotState.isShooting())
            ? RotationsPerSecond.of(0.25).in(RadiansPerSecond)
            : MaxAngularRate;
  }

  public Pose2d getPoseToResetTo() {

    Pose2d resetPose = AllianceFlipUtil.apply(new Pose2d(3.5, 4, new Rotation2d()));
    if (!RobotState.getInstance().isInAllianceZone()) {
      resetPose = AllianceFlipUtil.apply(new Pose2d(8.3, 4, new Rotation2d()));
    }
    return AllianceFlipUtil.apply(resetPose);
  }

  //
  // DRIVER STATION.
  //

  public HubTracker getTracker() {
    return tracker;
  }

  @Logged
  public double getMatchTime() {
    return DriverStation.getMatchTime();
  }
}
