package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.spindexer.Kicker;
import frc.robot.subsystems.spindexer.Spindexer;
import java.util.function.DoubleSupplier;

public class StaticShootCommand extends Command {

  private final Hood hood;
  private final Flywheel flywheel;
  private final Spindexer spindexer;
  private final Kicker kicker;
  private final DoubleSupplier flywheelRPMSupplier;
  private final DoubleSupplier hoodAngleSupplier;

  public StaticShootCommand(
      Hood hood,
      Flywheel flywheel,
      Spindexer spindexer,
      Kicker kicker,
      DoubleSupplier flywheelRPMSupplier,
      DoubleSupplier hoodAngleSupplier) {
    this.hood = hood;
    this.flywheel = flywheel;
    this.spindexer = spindexer;
    this.kicker = kicker;
    this.flywheelRPMSupplier = flywheelRPMSupplier;
    this.hoodAngleSupplier = hoodAngleSupplier;
  }

  public Command shoot() {
    return (new LogShootParamsCommand("Static Shoot ", flywheelRPMSupplier, hoodAngleSupplier))
        .andThen(
            flywheel
                .runAtSpeed(flywheelRPMSupplier)
                .alongWith(hood.goToSetpointRotationsDouble(hoodAngleSupplier))
                .alongWith(
                    Commands.waitUntil(() -> flywheel.isAtTarget())
                        .andThen(spindexer.run())
                        .andThen(kicker.run())));
  }

  public Command stop() {
    return flywheel.stopCommand().andThen(spindexer.stop()).andThen(kicker.stop());
  }
}
