// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import bearlib.fms.AllianceColor;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
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
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DynamicShootingCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;

public class Robot extends TimedRobot {
  private final Importance MINIMUM_IMPORTANCE = Importance.DEBUG;

  private Command m_autonomousCommand;

  // @Logged(importance = Importance.CRITICAL)
  private final RobotContainer m_robotContainer;

  private final CommandXboxController pilot = new CommandXboxController(0);

  private final CommandPS5Controller copilot = new CommandPS5Controller(1);

  @Logged private final Intake intake = new Intake();

  @Logged private final Spindexer spindexer = new Spindexer();

  // private final Turret turret = new Turret();

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  @Logged private final Flywheel flywheel = new Flywheel();

  @Logged private final Hood hood = new Hood();

  @Logged private final Climber climber = new Climber();

  private final ShootCommand shootCommand = new ShootCommand(hood, flywheel, spindexer, intake);

  private final DynamicShootingCommand dynamicShootingCommand =
      new DynamicShootingCommand(hood, flywheel, spindexer, intake, drivetrain);

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
  }

  @Override
  public void robotInit() {
    m_robotContainer.robotInit();
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

    // climber.setDefaultCommand(
    //    climber.manualDrive(() -> -MathUtil.applyDeadband(copilot.getRightY() / 2.0, 0.1)));

    drivetrain.registerTelemetry(telemetry::telemeterize);
  }

  public void configureBindings() {

    // pilot controlls

    pilot.leftTrigger().onTrue(intake.intakeOut()).onFalse(intake.intakeIn());

    pilot
        .rightTrigger()
        .onTrue(dynamicShootingCommand.shoot())
        .onFalse(dynamicShootingCommand.stop());

    pilot.a().onTrue(shootCommand.shoot()).onFalse(shootCommand.stop());

    // pilot.y().onTrue(hood.goToSetpointAngle(() -> Rotations.of(1.4)));

    // pilot.b().onTrue(hood.goToSetpointAngle(() -> Rotations.of(0)));

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
                        .withTargetDirection(RobotState.getInstance().getAngleToHub())));

   
    copilot.povUp().onTrue(climber.goToSetpoint(() -> Climber.Setpoint.Top));
    copilot.povDown().onTrue(climber.goToSetpoint(() -> Climber.Setpoint.Bottom));
    copilot.povLeft().onTrue(Commands.runOnce(() -> climber.getCurrentCommand().cancel()));
    copilot.povRight().onTrue(climber.calibrateZero());


  }
}
