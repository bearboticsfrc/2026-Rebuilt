package frc.robot.subsystems.shooter;

import bearlib.statemachine.State;
import bearlib.statemachine.StateMachineBase;
import edu.wpi.first.epilogue.Logged;
import frc.robot.RobotState;
import frc.robot.rebuilt.Copilot;
import frc.robot.rebuilt.Pilot;

public class FlywheelState extends StateMachineBase {

  DynamicShootingCalculator calculator = DynamicShootingCalculator.getInstance();

  RobotState robotState = RobotState.getInstance();

  public FlywheelState(Flywheel flywheel) {

    State idle = new State("Idle", () -> flywheel.runAtSpeed(600)).withEnd(() -> true);

    State shoot =
        new State(
                "Shoot",
                () -> (flywheel.runAtSpeed(() -> calculator.getParameters().flywheelVelocity())))
            .withEnd(() -> true);

    State flywheelIdle =
        new State("Flywheel Idle", () -> flywheel.stopCommand()).withEnd(flywheel::isStopped);

    State flywheel500 =
        new State("Flywheel 500", () -> flywheel.runAtSpeed(500)).withEnd(flywheel::isAtTarget);

    State flywheel1200 =
        new State("Flywheel 1.2k", () -> flywheel.runAtSpeed(1200)).withEnd(flywheel::isAtTarget);

    State flywheel3700 =
        new State("Flywheel 3.7k", () -> flywheel.runAtSpeed(3700)).withEnd(flywheel::isAtTarget);

    idle.to(shoot).condition(Pilot.shoot()::getAsBoolean);

    shoot.to(idle).condition(Pilot.shoot().negate()::getAsBoolean);

    flywheelIdle.global().condition(Copilot.flywheelIdle()::getAsBoolean);

    flywheel500.global().condition(Copilot.flywheel1200()::getAsBoolean);

    flywheel1200.global().condition(Copilot.flywheel1200()::getAsBoolean);

    flywheel3700.global().condition(Copilot.flywheel3700()::getAsBoolean);

    initState(idle);

    configure(idle, shoot, flywheelIdle, flywheel500, flywheel1200, flywheel3700);
  }

  /** Signals if the shooter is ready. */
  @Logged
  public boolean shooterReady() {
    return currentState() == "Shoot" && current().isComplete();
  }
}
