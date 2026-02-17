// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import bearlib.fms.AllianceColor;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.TrajectoryCalculator.ShotCalculator;
import frc.robot.subsystems.Turret.Turret;

public class Robot extends TimedRobot {
  private final Importance MINIMUM_IMPORTANCE = Importance.CRITICAL;

  private Command m_autonomousCommand;

  // @Logged(importance = Importance.CRITICAL)
  private final RobotContainer m_robotContainer;

  private final CommandXboxController pilot = new CommandXboxController(0);

  private final CommandXboxController copilot = new CommandXboxController(0);

  private final Intake intake = new Intake();

  private final ShooterCommand shooter = new ShooterCommand();

  private final Spindexer spindexer = new Spindexer();

  private final Turret turret = new Turret();

  private final Climber climber = new Climber();

  private final ShotCalculator shotCalculator = new ShotCalculator(drivetrain.getPose(), drivetrain.getChassisSpeeds());

  public Robot() {
    m_robotContainer = new RobotContainer();
    configureLogging();
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

  public void configureDefaultCommands(){
    //set default turret rotation
    double[] shotCalculations = shotCalculator.ShootOnMoveSolver(shotCalculator.targetLocation());
    turret.setAngle(Degrees.of(shotCalculations[3]));
  }


  public void configureBindings() {

    // pilot controlls
    pilot.rightBumper().whileTrue(intake.extendReverse());
    
    pilot.leftTrigger().whileTrue(intake.extendRun());
    
    pilot.y().onTrue(climber.climb());
    
    pilot.x().onTrue(climber.decend());

    pilot.rightTrigger().whileTrue(shooter.shoot());

    // copilot controlls
    copilot.leftTrigger().onTrue(Commands.run(() -> pilot.setRumble(RumbleType.kBothRumble, 1000000.0)));

  }
}
