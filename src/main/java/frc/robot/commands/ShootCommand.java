package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;

public class ShootCommand extends Command {

  private final Hood hood;
  private final Flywheel flywheel;
  private final Spindexer spindexer;

  public ShootCommand(Hood hood, Flywheel flywheel, Spindexer spindexer) {
    this.hood = hood;
    this.flywheel = flywheel;
    this.spindexer = spindexer;
  }

  public Command shoot() {
    return flywheel
        .runAtSpeed(3600)
        .alongWith(hood.goToSetpointAngle(() -> Rotations.of(.6)))
        .alongWith(
            Commands.waitUntil(() -> flywheel.isAtTarget())
                .andThen(spindexer.runSpindexer())
                .andThen(spindexer.runTower()));
  }

  public Command stop() {
    return flywheel.stopCommand().andThen(spindexer.stopMotorsCommand());
  }
}
