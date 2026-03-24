package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.DynamicShootingCalculator;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;
import lombok.Getter;

public class DynamicShootingCommand {

  private final Hood hood;
  private final Flywheel flywheel;
  private final Spindexer spindexer;
  private final Turret turret;
  private final DynamicShootingCalculator calculator = DynamicShootingCalculator.getInstance();

  @Getter private volatile double flywheelSpeed = 0;
  @Getter private volatile double hoodAngle = 0;

  public DynamicShootingCommand(Hood hood, Flywheel flywheel, Spindexer spindexer, Turret turret) {
    this.hood = hood;
    this.flywheel = flywheel;
    this.spindexer = spindexer;
    this.turret = turret;
  }

  public Command shoot() {
    return Commands.runOnce(() -> RobotState.getInstance().setShooting(true))
        .alongWith(
            flywheel
                .runAtSpeed(() -> calculator.getParameters().flywheelVelocity())
                .alongWith(
                    hood.goToSetpointRotationsDouble(() -> calculator.getParameters().hoodAngle()))
                .alongWith(
                    Commands.waitUntil(() -> flywheel.isAtTarget()).andThen(spindexer.run())));
  }

  public Command stop() {
    return Commands.runOnce(() -> RobotState.getInstance().setShooting(false))
        .alongWith(
            flywheel.stopCommand().alongWith(spindexer.stop()).alongWith(hood.stopCommand()));
  }
}
