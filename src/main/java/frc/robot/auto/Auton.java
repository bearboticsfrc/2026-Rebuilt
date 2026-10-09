package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.intake.IntakeState;
import frc.robot.subsystems.shooter.FlywheelState;
import frc.robot.subsystems.spindexer.SpindexerState;

public class Auton {

  private final SendableChooser<Command> autoChooser;

  private final FlywheelState flywheelState;
  private final SpindexerState spindexerState;
  private final IntakeState intakeState;

  public Auton(
      FlywheelState flywheelState, SpindexerState spindexerState, IntakeState intakeState) {

    this.flywheelState = flywheelState;
    this.spindexerState = spindexerState;
    this.intakeState = intakeState;

    // Setup Commands & EventTriggers.

    EventTrigger shoot = new EventTrigger("SHOOT");
    EventTrigger intake = new EventTrigger("INTAKE");
    EventTrigger stopShoot = new EventTrigger("STOPSHOOT");

    NamedCommands.registerCommand("SHOOT", shootCommand());
    NamedCommands.registerCommand("STOP_SHOOT", stopShootCommand());
    NamedCommands.registerCommand("STOPROLLERS", stopRollersCommand());
    NamedCommands.registerCommand("OSC", oscCommand());
    NamedCommands.registerCommand("RAMP", ramp());

    shoot.onTrue(shootCommand());
    intake.onTrue(intakeCommand());
    stopShoot.onTrue(stopShootCommand());

    autoChooser = AutoBuilder.buildAutoChooser("D"); // Default auto middle.
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  /** Returns the specific Command for the selected auto. */
  public Command getAutonomousCommand() {
    Command auto = autoChooser.getSelected();

    if (auto != null) {
      return auto;
    }
    return new PrintCommand("AUTO COMMAND IS NULL!!!");
  }

  /** Path planner auto chooser. */
  public SendableChooser<Command> getAutoChooser() {
    return autoChooser;
  }

  /** Sets flywheel state to shoot. Sets spindexer state to run. */
  private Command shootCommand() {
    return flywheelState
        .setState("Shoot")
        .andThen(Commands.waitSeconds(.65))
        .until(() -> flywheelState.shooterReady())
        .andThen(spindexerState.setState("Run"));
  }

  /** Sets intake state to intake. */
  private Command intakeCommand() {
    return intakeState.setState("Intake");
  }

  /** Sets the flywheel state to idle. Sets the spindexer state to idle. */
  private Command stopShootCommand() {
    return flywheelState.setState("Idle").alongWith(spindexerState.setState("Idle"));
  }

  /** Sets the intake state to rollers idle. Will leave slider extended. */
  private Command stopRollersCommand() {
    return intakeState.setState("Retract");
  }

  /** Oscillates the intake. */
  private Command oscCommand() {
    return intakeState.setState("Oscillate");
  }

  private Command ramp() {
    return flywheelState.setState("Idle");
  }
}
