package frc.robot.commands;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;
import lombok.Getter;

public class InterpolatedShootCommand {

  private final Hood hood;
  private final Flywheel flywheel;
  private final Spindexer spindexer;

  @Getter private volatile double flywheelSpeed = 0;
  @Getter private volatile double hoodAngle = 0;

  private static final InterpolatingDoubleTreeMap flywheelSpeedMap =
      new InterpolatingDoubleTreeMap();

  private static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();

  static {
    flywheelSpeedMap.put(1.8, 2300.0);
    flywheelSpeedMap.put(2.55, 2500.0);
    flywheelSpeedMap.put(3.0, 2650.0);
    flywheelSpeedMap.put(3.5, 2700.0);
    flywheelSpeedMap.put(4.0, 2800.0);
    flywheelSpeedMap.put(5.2, 3150.0);

    hoodAngleMap.put(1.8, 0.1);
    hoodAngleMap.put(2.55, 0.2);
    hoodAngleMap.put(3.0, 0.25);
    hoodAngleMap.put(3.5, 0.4);
    hoodAngleMap.put(4.0, 0.5);
    hoodAngleMap.put(5.2, 0.6);
  }

  private final Notifier solutionNotifier = new Notifier(this::calculateShootSolution);

  public InterpolatedShootCommand(Hood hood, Flywheel flywheel, Spindexer spindexer) {
    this.hood = hood;
    this.flywheel = flywheel;
    this.spindexer = spindexer;
  }

  private void calculateShootSolution() {
    double distance = RobotState.getInstance().getDistanceToHub();
    flywheelSpeed = flywheelSpeedMap.get(distance);
    hoodAngle = hoodAngleMap.get(distance);
  }

  public Command shoot() {

    return Commands.runOnce(
            () -> {
              calculateShootSolution();
              solutionNotifier.startPeriodic(.02);
            })
        .andThen(
            flywheel
                .runAtSpeed(this::getFlywheelSpeed)
                .alongWith(hood.goToSetpointRotationsDouble(this::getHoodAngle))
                .alongWith(
                    new LogShootParamsCommand(
                        "Interpolated Shoot", this::getFlywheelSpeed, this::getHoodAngle))
                .alongWith(
                    Commands.waitUntil(() -> flywheel.isAtTarget())
                        .andThen(spindexer.runSpindexer())
                        .andThen(spindexer.runTower())))
        .finallyDo(() -> solutionNotifier.stop());
  }

  public Command stop() {
    return flywheel
        .stopCommand()
        .alongWith(spindexer.stopMotorsCommand())
        .alongWith(hood.stopCommand())
        .andThen(Commands.runOnce(() -> solutionNotifier.stop()));
  }
}
