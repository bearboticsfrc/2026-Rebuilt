// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import bearlib.fms.AllianceColor;
import bearlib.util.TunableNumber;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.commands.InterpolatedShootCommand;
import frc.robot.field.Field;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DynamicShootingCalculator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.turret.Turret;

public class Robot extends TimedRobot {
  private final Importance MINIMUM_IMPORTANCE = Importance.DEBUG;

  private Command m_autonomousCommand;

  // @Logged(importance = Importance.CRITICAL)
  private final RobotContainer m_robotContainer;

  private final CommandXboxController pilot = new CommandXboxController(0);

  private final CommandPS5Controller copilot = new CommandPS5Controller(1);

  @Logged private final Intake intake = new Intake();

  @Logged private final IntakeArm intakeArm = new IntakeArm();

  @Logged private final Spindexer spindexer = new Spindexer();

  @Logged private final Turret turret = new Turret();

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  @Logged private final Flywheel flywheel = new Flywheel();

  @Logged private final Hood hood = new Hood();

  @Logged private final Climber climber = new Climber();

  // private final ShootCommand shootCommand = new ShootCommand(hood, flywheel, spindexer, intake);

  // private final DynamicShootingCommand dynamicShootingCommand =
  //     new DynamicShootingCommand(hood, flywheel, spindexer, intake, drivetrain);

  private TunableNumber rpm = new TunableNumber("RPM", 3600, () -> this.getTuningMode());

  private TunableNumber angle = new TunableNumber("Angle", .6, () -> this.getTuningMode());

  private final InterpolatedShootCommand shootCommand =
      new InterpolatedShootCommand(hood, flywheel, spindexer);

  // private final ShooterCommand shooter = new ShooterCommand(hood, flywheel, drivetrain);

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

  private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private final Telemetry telemetry = new Telemetry(MaxSpeed);

  public Robot() {
    m_robotContainer = new RobotContainer();

    configureLogging();
    configureBindings();
    configureDefaultCommands();

    DriverStation.silenceJoystickConnectionWarning(true);
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
  public void robotInit() {
    m_robotContainer.robotInit();

    // creating named commands for pathplanner auto builder
    NamedCommands.registerCommand(
        "Intake", new ScheduleCommand(intake.runRoller().andThen(intakeArm.extend())));
    NamedCommands.registerCommand(
        "StopIntake", new ScheduleCommand(intake.stopRollerCommand().andThen(intakeArm.retract())));
    NamedCommands.registerCommand("Shoot", (shootCommand.shoot()));
    NamedCommands.registerCommand("StopShoot", (shootCommand.stop()));
    NamedCommands.registerCommand("RaiseClimb", new ScheduleCommand(climber.raise()));
    NamedCommands.registerCommand("LowerClimb", new ScheduleCommand(climber.lower()));
  }

  @Override
  public void autonomousInit() {
    m_robotContainer.autonomousInit();
    CommandScheduler.getInstance().schedule(climber.calibrateZero());

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_robotContainer.teleopInit();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void disabledPeriodic() {
    m_robotContainer.disabledPeriodic();
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

    drivetrain.registerTelemetry(telemetry::telemeterize);
  }

  public void configureBindings() {

    // pilot controlls

    pilot
        .leftTrigger()
        .onTrue(intake.runRoller().andThen(intakeArm.extend()))
        .onFalse(
            intakeArm
                .retract()
                .alongWith(
                    Commands.waitSeconds(1)
                        .andThen(intake.stopRollerCommand())
                        .andThen(Commands.waitSeconds(2))));

    pilot.rightTrigger().onTrue(shootCommand.shoot()).onFalse(shootCommand.stop());

    pilot.a().whileTrue(new DriveToPoseCommand(drivetrain, () -> Field.getMyOutputPose()));

    // pilot.a().onTrue(shootCommand.shoot()).onFalse(shootCommand.stop());

    // copilot controlls
    copilot
        .L2()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    driveFacingAngle
                        .withVelocityX(-pilot.getLeftY() * MaxSpeed)
                        .withVelocityY(-pilot.getLeftX() * MaxSpeed)
                        .withHeadingPID(18, 0, .1)
                        .withTargetDirection(
                            DynamicShootingCalculator.getInstance()
                                .getParameters()
                                .turretAngle())));

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
    copilot.circle().onTrue(turret.setAngle(Rotations.of(-.20)));
    copilot.square().onTrue(turret.setAngle(Rotations.of(.20)));
  }
}
