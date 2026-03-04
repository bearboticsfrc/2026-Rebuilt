package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DynamicShootingCalculator;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;
import lombok.Getter;

public class DynamicShootingCommand{

  private final Hood hood;
  private final Flywheel flywheel;
  private final Spindexer spindexer;
  private final DynamicShootingCalculator calculator = DynamicShootingCalculator.getInstance(); 

  @Getter private volatile double flywheelSpeed = 0;
  @Getter private volatile double hoodAngle = 0;

  public DynamicShootingCommand(Hood hood, Flywheel flywheel, Spindexer spindexer) {
    this.hood = hood;
    this.flywheel = flywheel;
    this.spindexer = spindexer;
  }

  public Command shoot() {
            return flywheel.runAtSpeed(()-> calculator.getParameters().flywheelVelocity())
                .alongWith(hood.goToSetpointRotationsDouble(()-> calculator.getParameters().hoodAngle()))
                .alongWith(
                    Commands.waitUntil(() -> flywheel.isAtTarget())
                        .andThen(spindexer.runSpindexer())
                        .andThen(spindexer.runTower()));
  }

  public Command stop() {
    return flywheel
        .stopCommand()
        .alongWith(spindexer.stopMotorsCommand())
        .alongWith(hood.stopCommand());
  }
}
