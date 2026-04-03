// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import bearlib.fms.AllianceColor;
import bearlib.fms.AllianceReadyListener;
import bearlib.util.TunableNumber;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import dev.doglog.DogLog;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.geometry.Pose2d;
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
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoClimbCommand;
import frc.robot.commands.DynamicShootingCommand;
import frc.robot.commands.InterpolatedShootCommand;
import frc.robot.commands.StaticShootCommand;
import frc.robot.field.AllianceFlipUtil;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DynamicShootingCalculator;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.*;
import frc.robot.subsystems.intake.Rollers;
import frc.robot.subsystems.intake.Slider;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.spindexer.Kicker;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;
import frc.robot.test.SelfTest;
import frc.robot.util.HubTracker;
import frc.robot.vision.VisionConstants;
import frc.robot.vision.VisionSystem;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.Telemetry.PrintPriority;
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

  private Command m_autonomousCommand;

  private final SendableChooser<Command> autoChooser;

  private final CommandXboxController pilot = new CommandXboxController(0);

  private final CommandPS5Controller copilot = new CommandPS5Controller(1);

  @Logged private final HubTracker tracker;

  @Logged private final Rollers rollers;

  @Logged private final Slider slider;

  @Logged private final Spindexer spindexer;
  @Logged private final Kicker kicker;

  @Logged @Getter private final Turret turret;

  @Logged @Getter public final VisionSystem vision;

  @Logged private final CommandSwerveDrivetrain drivetrain;

  @Logged private final Flywheel flywheel;

  @Logged private final Hood hood;

  @Logged private final Climber climber;

  @Logged private DynamicShootingCalculator calculator;

  @Logged private RobotState robotState = RobotState.getInstance();

  @Logged private StateMachine stateMachine;

  @Getter public Field2d field2d = new Field2d();

  private Command introspectedAutoCommand;

  @Logged(name = "Auto Start Pose", importance = Importance.CRITICAL)
  private Pose2d autoStartPose;

  // for shot tuning
  private TunableNumber rpm = new TunableNumber("RPM", 3600, () -> this.getTuningMode());
  private TunableNumber angle = new TunableNumber("Angle", .6, () -> this.getTuningMode());

  private final InterpolatedShootCommand interpolatedShootCommand;

  private final DynamicShootingCommand dynamicShootingCommand;

  private final StaticShootCommand staticShootCommand;

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

  private final SwerveRequest.SwerveDriveBrake breakMode = new SwerveRequest.SwerveDriveBrake();

  private final DriveTelemetry driveTelemetry = new DriveTelemetry();

  private final AutoClimbCommand autoClimbCommand;

  private final SelfTest selfTest;

  public Robot() {
    instance = this;
    Telemetry.start(true, false, PrintPriority.NORMAL);
    tracker = new HubTracker();
    rollers = new Rollers();
    slider = new Slider();
    spindexer = new Spindexer();
    kicker = new Kicker();
    turret = new Turret();
    drivetrain = TunerConstants.createDrivetrain();

    flywheel = new Flywheel();
    hood = new Hood();
    climber = new Climber();

    calculator = DynamicShootingCalculator.getInstance();

    // spectrumVision = new SpectrumVision(new SpectrumVision.VisionConfig());

    vision =
        new VisionSystem(
            Arrays.asList(VisionConstants.REAR_CAMERA, VisionConstants.RIGHT_CAMERA),
            false,
            drivetrain,
            () -> turret.getPositionDegrees(),
            () -> drivetrain.getState().Speeds.omegaRadiansPerSecond);

    Telemetry.print("All subsystems Initialized");

    stateMachine = new StateMachine(pilot, copilot, flywheel, slider, climber);

    interpolatedShootCommand = new InterpolatedShootCommand(hood, flywheel, spindexer, kicker);

    dynamicShootingCommand = new DynamicShootingCommand(hood, flywheel, spindexer, kicker, turret);

    staticShootCommand = new StaticShootCommand(hood, flywheel, spindexer, kicker, rpm, angle);

    autoClimbCommand = new AutoClimbCommand(drivetrain, climber);

    selfTest =
        new SelfTest(
            rollers, flywheel, hood, spindexer, kicker, turret, slider, climber, drivetrain);

    registerPathplannerCommands();

    autoChooser = AutoBuilder.buildAutoChooser("O Bump");
    SmartDashboard.putData("Auto Mode", autoChooser);

    configureLogging();
    configureBindings();
    selfTest.bindTriggers();
    configureDefaultCommands();
    // configureStateMachine();

    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    AllianceColor.addListener(this);

    DriverStation.silenceJoystickConnectionWarning(true);

    // Set the scheduler to log when a command initializes, interrupts, or finishes
    CommandScheduler scheduler = CommandScheduler.getInstance();
    scheduler.onCommandInitialize(
        command -> DogLog.log("Misc/Robot Status", "Initialized: " + command.getName()));
    scheduler.onCommandInterrupt(
        (command, interrupter) ->
            DogLog.log(
                "Misc/Robot Status",
                "Interrupted: "
                    + command.getName()
                    + " , by: "
                    + (interrupter.isPresent() ? interrupter.get().getName() : "")));
    scheduler.onCommandFinish(
        command -> DogLog.log("Misc/Robot Status", "Finished: " + command.getName()));

    // DogLog.setPdh(new PowerDistribution());
  }

  public CommandSwerveDrivetrain getSwerve() {
    return drivetrain;
  }

  public Flywheel getFlywheel() {
    return flywheel;
  }

  public HubTracker getTracker() {
    return tracker;
  }

  private boolean getTuningMode() {
    return true;
  }

  @Logged
  public double getDistanceToHub() {
    return RobotState.getInstance().getDistanceToHub();
  }

  // TODO: Make max speed relative to distance to hub, so that we can be more precise when close to
  // the hub and faster when far away
  public Supplier<Double> getMaxLinearVelocity() {
    double distanceToHub = robotState.getDistanceToHub();
    return () -> (robotState.isShooting()) ? 1.25 - ((distanceToHub / 5.5) * 0.5) : MaxSpeed;
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

  public void registerPathplannerCommands() {

    // named commands for autonomous
    NamedCommands.registerCommand(
        "Intake",
        new ScheduleCommand(
                rollers
                    .run()
                    .alongWith(slider.extend())
                    .alongWith(spindexer.oscillate())
                    .withName("ParallelRollerArm"))
            .withName("ScheduleIntake"));

    NamedCommands.registerCommand(
        "StopIntake",
        new ScheduleCommand(
                slider
                    .retract()
                    .alongWith(spindexer.stop())
                    .alongWith(kicker.stop())
                    .alongWith(rollers.stop())
                    .withName("ParallelRetractStop"))
            .withName("ScheduleStopIntake"));

    NamedCommands.registerCommand(
        "Shoot",
        new ScheduleCommand(
                dynamicShootingCommand.shoot().withTimeout(5).withName("TurretAndShoot5"))
            .withName("ScheduleShoot"));

    NamedCommands.registerCommand(
        "ShootRoll",
        new ScheduleCommand(
                dynamicShootingCommand
                    .shoot()
                    .withTimeout(5)
                    .alongWith(rollers.runSlow())
                    .withName("ShootRoll"))
            .withName("ScheduleShootRoll"));

    NamedCommands.registerCommand(
        "StopShoot",
        new ScheduleCommand(dynamicShootingCommand.stop().withName("StopShoot"))
            .withName("ScheduleStopShoot"));

    NamedCommands.registerCommand("RaiseClimb", new ScheduleCommand(climber.raise()));

    NamedCommands.registerCommand("LowerClimb", new ScheduleCommand(climber.lower()));
  }

  // set turret
  private Command getTurretCommand() {
    return turret
        .setAngle(
            () -> calculator.getParameters().turretAngle().getMeasure(),
            () -> calculator.getParameters().turretVelocity())
        .withName("TurretCommand");
  }

  @Override
  public void autonomousInit() {
    vision.updateCameraSettings();

    CommandScheduler.getInstance().schedule(slider.retract());
    CommandScheduler.getInstance().schedule(climber.calibrateZero());

    m_autonomousCommand = getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  public Command getAutonomousCommand() {
    /* Run the path selected from the auto chooser */
    return autoChooser.getSelected();
  }

  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    vision.updateCameraSettings();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  /** Disabled periodic which updates the autonomous starting pose. */
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

    // default drive request
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        -pilot.getLeftY()
                            * getMaxLinearVelocity()
                                .get()) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        -pilot.getLeftX()
                            * getMaxLinearVelocity().get()) // Drive left with negative X (left)
                    .withRotationalRate(
                        -pilot.getRightX()
                            * getMaxAngularVelocity()
                                .get()) // Drive counterclockwise with negative X (left)
            ));

    drivetrain.registerTelemetry(driveTelemetry::telemeterize);

    turret.setDefaultCommand(getTurretCommand());
  }

  public void configureBindings() {

    // pilot controlls
    pilot
        .leftTrigger()
        .onTrue(rollers.run().alongWith(slider.extend()).withName("ParallelIntake"))
        .onFalse(
            slider
                .retract()
                .alongWith(
                    Commands.waitSeconds(1)
                        .andThen(rollers.stop())
                        .andThen(Commands.waitSeconds(2)))
                .withName("ParallelRetractIntake"));
    pilot
        .rightTrigger()
        .onTrue(dynamicShootingCommand.shoot())
        .onFalse(dynamicShootingCommand.stop());

    // copilot controlls
    copilot.povUp().onTrue(climber.raise());

    copilot.povDown().onTrue(climber.lower());

    copilot.povRight().onTrue(climber.calibrateZero());

    // copilot.circle().whileTrue(autoClimbCommand.climb());

    copilot
        .povLeft()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (climber.getCurrentCommand() != null) climber.getCurrentCommand().cancel();
                }));

    copilot.triangle().onTrue(turret.setAngle(Rotations.of(0)));
    copilot
        .circle()
        .onTrue(Commands.runOnce(() -> drivetrain.getPigeon2().reset()).ignoringDisable(true));

    copilot.cross().onTrue(Commands.runOnce(() -> vision.resetToFrontCameraPose()));

    copilot.L1().onTrue(turret.setAngle(Rotations.of(-.25)));

    copilot.R1().onTrue(turret.setAngle(Rotations.of(.25)));

    copilot.L2().toggleOnTrue(Commands.idle(turret));

    copilot.R2().whileTrue(drivetrain.applyRequest(() -> breakMode));

    copilot
        .square()
        .whileTrue(spindexer.reverse().andThen(kicker.reverse()))
        .onFalse(spindexer.stop().alongWith(kicker.stop()));
  }

  public void configureStateMachine() {
    // shooter
    stateMachine
        .onEnter(ShootStates.RAMP)
        .onTrue(
            flywheel
                .runAtSpeed(calculator.getParameters().flywheelVelocity())
                .alongWith(
                    hood.goToSetpointRotationsDouble(
                        () -> calculator.getParameters().hoodAngle())));

    stateMachine
        .onExit(ShootStates.SHOOT)
        .onTrue(flywheel.stopCommand().alongWith(hood.stopCommand()));

    // spindexer
    stateMachine.onEnter(ShootStates.SHOOT).onTrue(kicker.run().andThen(spindexer.run()));

    stateMachine.onExit(ShootStates.SHOOT).onTrue(kicker.stop().alongWith(spindexer.stop()));

    // intake

    stateMachine.onEnter(IntakeStates.RETRACT).onTrue(slider.retract().alongWith(rollers.stop()));

    stateMachine.onEnter(IntakeStates.EXTENDING).onTrue(slider.extend());

    stateMachine.onEnter(IntakeStates.EXTENDED).onTrue(rollers.run());

    stateMachine.onEnter(IntakeStates.OSCILLATE).onTrue(slider.oscillate());

    // turret
    stateMachine
        .onEnter(TurretStates.TRACK)
        .onTrue(
            turret.setAngle(
                () -> calculator.getParameters().turretAngle().getMeasure(),
                () -> calculator.getParameters().turretVelocity()));

    // climber
    stateMachine.onEnter(ClimbStates.EXTENDING).onTrue(climber.raise());

    stateMachine.onEnter(ClimbStates.RETRACTING).onTrue(climber.lower());

    stateMachine.onEnter(ClimbStates.RETRACTED).onTrue(climber.calibrateZero());
  }

  // public void bindDriveSysidTriggers() {
  //   pilot.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
  //   pilot.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));

  //   /*
  //    * Joystick Y = quasistatic forward
  //    * Joystick A = quasistatic reverse
  //    * Joystick B = dynamic forward
  //    * Joystick X = dyanmic reverse
  //    */

  //   pilot.y().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
  //   pilot.a().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
  //   pilot.b().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
  //   pilot.x().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
  // }

  public void setupSmartDashboardData() {
    SmartDashboard.putData("Field2d", field2d);
  }

  private boolean initialPoseSet = false;

  @Override
  public void updateAlliance(Alliance alliance) {
    if (!initialPoseSet) {
      Command firstAuto = autoChooser.getSelected();
      System.out.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!FirstAuto: " + firstAuto.getName());
      drivetrain.resetPose(AllianceFlipUtil.apply(((PathPlannerAuto) firstAuto).getStartingPose()));
      vision.resetPose();
      initialPoseSet = true;
    }
  }
}
