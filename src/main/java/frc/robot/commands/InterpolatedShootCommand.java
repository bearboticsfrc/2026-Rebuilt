package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.spindexer.Kicker;
import frc.robot.subsystems.spindexer.Spindexer;
import lombok.Getter;

public class InterpolatedShootCommand {

  private final Hood hood;
  private final Flywheel flywheel;
  private final Spindexer spindexer;
  private final Kicker kicker;

  @Getter private volatile double flywheelSpeed = 0;
  @Getter private volatile double hoodAngle = 0;

  private static final InterpolatingDoubleTreeMap flywheelSpeedMap =
      new InterpolatingDoubleTreeMap();

  private static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();

  static {

    // flywheelSpeedMap.put(distance, speed)

    flywheelSpeedMap.put(1.17, 2400.0);
    flywheelSpeedMap.put(2.0, 2425.0);
    flywheelSpeedMap.put(2.67, 2625.0);
    flywheelSpeedMap.put(3.0, 2700.0);
    flywheelSpeedMap.put(3.57, 2900.0);
    flywheelSpeedMap.put(4.0, 3100.0);
    flywheelSpeedMap.put(4.67, 3250.0);
    flywheelSpeedMap.put(5.5, 3650.0);

    hoodAngleMap.put(1.17, 0.0);
    hoodAngleMap.put(2.0, 0.0);
    hoodAngleMap.put(2.67, 0.0);
    hoodAngleMap.put(3.0, 0.0);
    hoodAngleMap.put(3.57, 0.0);
    hoodAngleMap.put(4.0, 0.0);
    hoodAngleMap.put(4.67, 0.0);
    hoodAngleMap.put(5.5, 0.0);
  }

  private final Notifier solutionNotifier = new Notifier(this::calculateShootSolution);

  public InterpolatedShootCommand(
      Hood hood, Flywheel flywheel, Spindexer spindexer, Kicker kicker) {
    this.hood = hood;
    this.flywheel = flywheel;
    this.spindexer = spindexer;
    this.kicker = kicker;
  }

  private void calculateShootSolution() {
    double distance = MathUtil.clamp(RobotState.getInstance().getDistanceToHub(), 1.25, 5.5);
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
                        .andThen(spindexer.run())
                        .andThen(kicker.run())))
        .finallyDo(() -> solutionNotifier.stop());
  }

  public Command stop() {
    return flywheel
        .stopCommand()
        .alongWith(spindexer.stop())
        .alongWith(kicker.stop())
        .alongWith(hood.stopCommand());
  }
}
