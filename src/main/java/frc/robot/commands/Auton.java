package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.IntakeState;
import frc.robot.subsystems.shooter.FlywheelState;
import frc.robot.subsystems.shooter.HoodState;
import frc.robot.subsystems.spindexer.SpindexerState;

public class Auton {

  private final SendableChooser<Command> autoChooser;

  private final SpindexerState spindexerState;
  private final IntakeState intakeState;
  private final FlywheelState flywheelState;
  private final HoodState hoodState;

  public Auton(
      SpindexerState spindexerState,
      IntakeState intakeState,
      FlywheelState flywheelState,
      HoodState hoodState) {

    this.spindexerState = spindexerState;
    this.intakeState = intakeState;
    this.flywheelState = flywheelState;
    this.hoodState = hoodState;

    EventTrigger shoot = new EventTrigger("SHOOT");
    EventTrigger intake = new EventTrigger("INTAKE");
    EventTrigger stopShoot = new EventTrigger("STOPSHOOT");

    NamedCommands.registerCommand("SHOOT", shootCommand());
    NamedCommands.registerCommand("STOPROLLERS", stopRollersCommand());

    shoot.onTrue(shootCommand());
    intake.onTrue(intakeCommand());
    stopShoot.onTrue(stopShootCommand());

    autoChooser = AutoBuilder.buildAutoChooser("MO");
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public SendableChooser<Command> getAutoChooser() {
    return autoChooser;
  }

  /**
   * Sets the flywheel state to ground, prevent decapitation. Sets flywheel state to shoot. Sets
   * spindexer state to run.
   */
  private Command shootCommand() {
    return Commands.runOnce(() -> hoodState.setState("Ground"))
        .alongWith(
            Commands.runOnce(() -> flywheelState.setState("Shoot"))
                .alongWith(Commands.run(() -> spindexerState.setState("Run"))));
  }

  /** Sets intake state to intake. */
  private Command intakeCommand() {
    return Commands.runOnce(() -> intakeState.setState("Intake"));
  }

  /** Sets the flywheel state to idle. Sets the spindexer state to idle. */
  private Command stopShootCommand() {
    return Commands.runOnce(() -> flywheelState.setState("Idle"))
        .alongWith(Commands.runOnce(() -> spindexerState.setState("Idle")));
  }

  /** Sets the intake state to rollers idle. */
  private Command stopRollersCommand() {
    return Commands.runOnce(() -> intakeState.setState("Rollers Idle"));
  }
}
