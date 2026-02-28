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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.shooter.Flywheel;

public class Robot extends TimedRobot {
  private final Importance MINIMUM_IMPORTANCE = Importance.DEBUG;

  private Command m_autonomousCommand;

  // @Logged(importance = Importance.CRITICAL)
  private final RobotContainer m_robotContainer;

  private final CommandXboxController pilot = new CommandXboxController(0);

  private final CommandXboxController copilot = new CommandXboxController(1);

  @Logged private final Intake intake = new Intake();

  @Logged private final Spindexer spindexer = new Spindexer();

  // private final Turret turret = new Turret();
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  private final Flywheel flywheel = new Flywheel();
  // private final Hood hood = new Hood();
  // private final Climber climber = new Climber();

  // private final ShotCalculator shotCalculator;

  // private final ShooterCommand shooter = new ShooterCommand(hood, flywheel, turret, drivetrain);

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

  public Robot() {
    m_robotContainer = new RobotContainer();

    // shotCalculator =
    //     new ShotCalculator(() -> drivetrain.getPose(), () -> drivetrain.getChassisSpeeds());

    configureLogging();
    configureBindings();
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
    CommandScheduler.getInstance().run();
  }

  @Override
  public void robotInit() {
    m_robotContainer.robotInit();
  }

  @Override
  public void autonomousInit() {
    m_robotContainer.autonomousInit();

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
    // set default turret rotation
    // double[] shotCalculations =
    // shotCalculator.ShootOnMoveSolver(shotCalculator.targetLocation());
    // turret.setAngle(Degrees.of(shotCalculations[3]));
  }

  public void configureBindings() {

    // pilot controlls
    // pilot.rightTrigger().whileTrue(shooter.shoot());

    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
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

    pilot.rightTrigger().onTrue(intake.intakeOut()).onFalse(intake.intakeIn());
    pilot.leftTrigger().onTrue(spindexer.index()).onFalse(spindexer.stopMotorsCommand());
    pilot.rightBumper().onTrue(flywheel.runFast()).onFalse(flywheel.stopCommand());

    // copilot controlls
  }
}
