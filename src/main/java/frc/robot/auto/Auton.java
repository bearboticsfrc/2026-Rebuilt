package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.intake.Rollers;
import frc.robot.subsystems.intake.Slider;
import frc.robot.subsystems.shooter.DynamicShootingCalculator;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.spindexer.Kicker;
import frc.robot.subsystems.spindexer.Spindexer;
import java.util.Set;

public class Auton {

  private final SendableChooser<Command> autoChooser;

  private final Spindexer spindexer;
  private final Flywheel flywheel;
  private final Hood hood;
  private final Slider slider;
  private final Rollers rollers;
  private final Kicker kicker;

  private final DynamicShootingCalculator calculator = DynamicShootingCalculator.getInstance();

  public Auton(
      Spindexer spindexer,
      Flywheel flywheel,
      Hood hood,
      Slider slider,
      Rollers rollers,
      Kicker kicker) {

    this.spindexer = spindexer;
    this.flywheel = flywheel;
    this.hood = hood;
    this.slider = slider;
    this.rollers = rollers;
    this.kicker = kicker;

    // Setup Commands & EventTriggers.

    EventTrigger shoot = new EventTrigger("SHOOT");
    EventTrigger intake = new EventTrigger("INTAKE");
    EventTrigger stopShoot = new EventTrigger("STOPSHOOT");

    NamedCommands.registerCommand("SHOOT", Commands.defer(this::shootCommand, Set.of()));
    NamedCommands.registerCommand(
        "STOPROLLERS", Commands.defer(this::stopRollersCommand, Set.of()));

    shoot.onTrue(Commands.defer(this::shootCommand, Set.of()));
    intake.onTrue(Commands.defer(this::intakeCommand, Set.of()));
    stopShoot.onTrue(Commands.defer(this::stopShootCommand, Set.of()));

    autoChooser = AutoBuilder.buildAutoChooser("MO"); // Default auto middle.
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

  /**
   * Sets the flywheel state to ground, prevent decapitation. Sets flywheel state to shoot. Sets
   * spindexer state to run.
   */
  private Command shootCommand() {
    return flywheel
        .runAtSpeed(() -> calculator.getParameters().flywheelVelocity())
        .alongWith(hood.goToSetpointRotationsDouble(() -> calculator.getParameters().hoodAngle()))
        .alongWith(spindexer.run())
        .alongWith(kicker.run());
  }

  /** Sets intake state to intake. */
  private Command intakeCommand() {
    return slider.extend().alongWith(rollers.run());
  }

  /** Sets the flywheel state to idle. Sets the spindexer state to idle. */
  private Command stopShootCommand() {
    return flywheel
        .stopCommand()
        .alongWith(hood.ground())
        .alongWith(spindexer.stop())
        .alongWith(kicker.stop());
  }

  /** Sets the intake state to rollers idle. Will leave slider extended. */
  private Command stopRollersCommand() {
    return rollers.stop();
  }
}
