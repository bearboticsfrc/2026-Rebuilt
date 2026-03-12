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
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.commands.DynamicShootingCommand;
import frc.robot.commands.InterpolatedShootCommand;
import frc.robot.field.AllianceFlipUtil;
import frc.robot.field.Field;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DynamicShootingCalculator;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.intake.IntakeArm;
import frc.robot.subsystems.intake.Rollers;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.HubTracker;
import frc.robot.vision.SpectrumVision;
import frc.robot.vision.VisionConstants;
import frc.robot.vision.VisionSystem;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.Telemetry.PrintPriority;
import java.util.Arrays;
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

  // @Logged(importance = Importance.CRITICAL)

  private final CommandXboxController pilot = new CommandXboxController(0);

  private final CommandPS5Controller copilot = new CommandPS5Controller(1);

  @Logged private final HubTracker tracker;

  @Logged private final Rollers rollers = new Rollers();

  @Logged private final IntakeArm intakeArm;

  @Logged private final Spindexer spindexer;

  @Logged @Getter private final Turret turret;

  @Logged @Getter public final VisionSystem vision;
  @Logged @Getter public final SpectrumVision spectrumVision;

  @Logged private final CommandSwerveDrivetrain drivetrain;

  @Logged private final Flywheel flywheel;

  @Logged private final Hood hood;

  @Logged private final Climber climber;

  @Logged private DynamicShootingCalculator calculator = DynamicShootingCalculator.getInstance();

  @Logged private RobotState robotState = RobotState.getInstance();

  @Getter public Field2d field2d = new Field2d();

  private Command introspectedAutoCommand;

  @Logged(name = "Auto Start Pose", importance = Importance.CRITICAL)
  private Pose2d autoStartPose;

  // for shot tuning
  private TunableNumber rpm = new TunableNumber("RPM", 3600, () -> this.getTuningMode());
  private TunableNumber angle = new TunableNumber("Angle", .6, () -> this.getTuningMode());

  private final InterpolatedShootCommand interpolatedShootCommand;

  private final DynamicShootingCommand dynamicShootingCommand;

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

  private final DriveTelemetry driveTelemetry = new DriveTelemetry(MaxSpeed);

  private final AutoClimbCommand autoClimbCommand;

  public Robot() {
    instance = this;
    Telemetry.start(true, false, PrintPriority.NORMAL);

    tracker = new HubTracker();
    intake = new Intake();
    intakeArm = new IntakeArm();
    spindexer = new Spindexer();
    turret = new Turret();
    drivetrain = TunerConstants.createDrivetrain();

    flywheel = new Flywheel();
    hood = new Hood();
    climber = new Climber();
    spectrumVision = new SpectrumVision(new SpectrumVision.VisionConfig());

    vision =
        new VisionSystem(
            Arrays.asList(VisionConstants.FRONT_CAMERA, VisionConstants.REAR_CAMERA), drivetrain);

    Telemetry.print("All subsystems Initialized");

    interpolatedShootCommand = new InterpolatedShootCommand(hood, flywheel, spindexer);

    dynamicShootingCommand = new DynamicShootingCommand(hood, flywheel, spindexer, turret);

    autoClimbCommand = new AutoClimbCommand(drivetrain, climber);

    registerPathplannerCommands();

    autoChooser = AutoBuilder.buildAutoChooser("OStart - Outpost - Climb");
    SmartDashboard.putData("Auto Mode", autoChooser);

    configureLogging();
    configureBindings();
    configureDefaultCommands();

    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    AllianceColor.addListener(this);

    DriverStation.silenceJoystickConnectionWarning(true);
  }

  public CommandSwerveDrivetrain getSwerve() {
    return drivetrain;
  }

  private boolean getTuningMode() {
    return true;
  }

  @Logged
  public double getDistanceToHub() {
    return RobotState.getInstance().getDistanceToHub();
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
    RobotState.getInstance().updatePose();
    CommandScheduler.getInstance().run();
    DynamicShootingCalculator.getInstance().clearLaunchingParameters();
  }

  @Override
  public void robotInit() {}

  public void registerPathplannerCommands() {
    // named commands for autonomous
    NamedCommands.registerCommand(
        "Intake", new ScheduleCommand(rollers.run().andThen(intakeArm.extend())));

    NamedCommands.registerCommand(
        "StopIntake", new ScheduleCommand(rollers.stop().andThen(intakeArm.retract())));

    NamedCommands.registerCommand(
        "Shoot",
        new ScheduleCommand(
            getTurretCommand().alongWith(dynamicShootingCommand.shoot().withTimeout(5))));

    NamedCommands.registerCommand(
        "ShootRoll",
        new ScheduleCommand(
                getTurretCommand().alongWith(dynamicShootingCommand.shoot().withTimeout(5)))
            .alongWith(rollers.runSlow()));

    NamedCommands.registerCommand(
        "StopShoot", new ScheduleCommand(turret.stop().alongWith(dynamicShootingCommand.stop())));

    NamedCommands.registerCommand("RaiseClimb", new ScheduleCommand(climber.raise()));

    NamedCommands.registerCommand("LowerClimb", new ScheduleCommand(climber.lower()));
  }

  // set turret
  private Command getTurretCommand() {
    return turret.setAngle(() -> calculator.getParameters().turretAngle().getMeasure());
  }

  @Override
  public void autonomousInit() {
    CommandScheduler.getInstance().schedule(intakeArm.retract());
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
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  /** Disabled periodic which updates the autonomous starting pose. */
  public void disabledPeriodic() {
    Command selectedAutoCommand = autoChooser.getSelected();

    if (introspectedAutoCommand != selectedAutoCommand
        && selectedAutoCommand instanceof PathPlannerAuto) {
      autoStartPose =
          AllianceFlipUtil.apply(((PathPlannerAuto) selectedAutoCommand).getStartingPose());
      introspectedAutoCommand = selectedAutoCommand;
      drivetrain.resetPose(autoStartPose);
      System.out.println("Setting autostartpose!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    }
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
                        -pilot.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        -pilot.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(
                        -pilot.getRightX()
                            * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));

    drivetrain.registerTelemetry(driveTelemetry::telemeterize);

    turret.setDefaultCommand(
        turret.setAngle(() -> calculator.getParameters().turretAngle().getMeasure()));
  }

  public void configureBindings() {

    // pilot controlls
    pilot
        .leftTrigger()
        .onTrue(rollers.run().andThen(intakeArm.extend()))
        .onFalse(
            intakeArm
                .retract()
                .alongWith(
                    Commands.waitSeconds(1)
                        .andThen(rollers.stop())
                        .andThen(Commands.waitSeconds(2))));

    pilot
        .rightTrigger()
        .onTrue(dynamicShootingCommand.shoot())
        .onFalse(dynamicShootingCommand.stop());

    pilot.a().whileTrue(new DriveToPoseCommand(drivetrain, () -> Field.getMyOutputPose()));

    // copilot controlls
    copilot.povUp().onTrue(climber.raise());

    copilot.povDown().onTrue(climber.lower());

    copilot
        .povLeft()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (climber.getCurrentCommand() != null) climber.getCurrentCommand().cancel();
                }));

    copilot.povRight().onTrue(climber.calibrateZero());

    copilot.triangle().onTrue(turret.setAngle(Rotations.of(0)));
    copilot.circle().whileTrue(autoClimbCommand.climb());
    copilot
        .square()
        .whileTrue(spindexer.reverseSpindexer().andThen(spindexer.reverseTower()))
        .onFalse(spindexer.stopMotorsCommand());

    copilot.cross().onTrue(Commands.runOnce(() -> vision.resetToFrontCameraPose()));

    copilot.L1().onTrue(turret.setAngle(Rotations.of(-.25)));

    copilot.R1().onTrue(turret.setAngle(Rotations.of(.25)));

    copilot.L2().toggleOnTrue(Commands.idle(turret));
  }

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
      initialPoseSet = true;
    }
  }
}
