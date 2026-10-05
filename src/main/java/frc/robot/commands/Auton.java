package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.intake.Rollers;
import frc.robot.subsystems.intake.Slider;

public class Auton {

  private final SendableChooser<Command> autoChooser;

  public Auton(DynamicShootingCommand shootCommand, Rollers rollers, Slider slider) {

    // Setup Commands & EventTriggers.
    EventTrigger shoot = new EventTrigger("SHOOT");
    EventTrigger intake = new EventTrigger("INTAKE");
    EventTrigger stopShoot = new EventTrigger("STOPSHOOT");

    intake.onTrue(rollers.run().alongWith(slider.extend()));
    shoot.onTrue(shootCommand.shoot());
    stopShoot.onTrue(shootCommand.stop());

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
}
