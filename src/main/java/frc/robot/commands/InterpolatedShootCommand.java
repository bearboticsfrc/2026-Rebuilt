package frc.robot.commands;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;

public class InterpolatedShootCommand {

  private final Hood hood;
  private final Flywheel flywheel;
  private final Spindexer spindexer;
  private final Intake intake;

  private static final InterpolatingDoubleTreeMap flywheelSpeedMap =
      new InterpolatingDoubleTreeMap();

  private static final InterpolatingDoubleTreeMap hoodAngleMapMap =
      new InterpolatingDoubleTreeMap();

  static {
    flywheelSpeedMap.put(1.5, 2000.0);
    flywheelSpeedMap.put(2.5, 2500.0);
    flywheelSpeedMap.put(4.75, 3000.0);

    hoodAngleMapMap.put(1.5, 0.5);
    hoodAngleMapMap.put(2.5, 0.6);
    hoodAngleMapMap.put(4.75, 0.7);
  }

  public InterpolatedShootCommand(
      Hood hood, Flywheel flywheel, Spindexer spindexer, Intake intake) {
    this.hood = hood;
    this.flywheel = flywheel;
    this.spindexer = spindexer;
    this.intake = intake;
  }

  public Command shoot() {
    return flywheel
        .runAtSpeed(() -> calculateFlywheelSpeed())
        .alongWith(hood.goToSetpointRotationsDouble(() -> calculateHoodAngle()))
        .alongWith(
            Commands.waitUntil(() -> flywheel.isAtTarget())
                .andThen(spindexer.runSpindexer())
                .andThen(spindexer.runTower())
                .andThen(intake.runMouthSlow()));
  }

  public Command stop() {
    return flywheel
        .stopCommand()
        .andThen(spindexer.stopMotorsCommand())
        .andThen(intake.stopMouthCommand());
  }

  private double calculateFlywheelSpeed() {
    double distance = RobotState.getInstance().getDistanceToHub();
    return flywheelSpeedMap.get(distance);
  }

  private double calculateHoodAngle() {
    double distance = RobotState.getInstance().getDistanceToHub();
    return hoodAngleMapMap.get(distance);
  }
}
