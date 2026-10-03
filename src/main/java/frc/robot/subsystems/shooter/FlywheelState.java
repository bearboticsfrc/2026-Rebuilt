package frc.robot.subsystems.shooter;

import frc.robot.RobotState;
import frc.robot.bearlib.statemachine.State;
import frc.robot.bearlib.statemachine.StateMachineBase;
import frc.robot.bearlib.util.TunableNumber;
import frc.robot.rebuilt.Pilot;

public class FlywheelState extends StateMachineBase {

  DynamicShootingCalculator calculator = DynamicShootingCalculator.getInstance();

  RobotState robotState = RobotState.getInstance();

  public FlywheelState(Flywheel flywheel, TunableNumber rpm) {

    State idle = new State("Idle", () -> flywheel.runAtSpeed(0.0));

    State shoot =
        new State(
                "Shoot",
                () -> flywheel.runAtSpeed(() -> calculator.getParameters().flywheelVelocity()))
            .withEnd(() -> true);

    State tune = new State("Tune", () -> flywheel.runAtSpeed(rpm));

    idle.to(tune).condition(Pilot.shoot()::getAsBoolean);

    tune.to(idle).condition(Pilot.shoot().negate()::getAsBoolean);

    initState(idle);

    configure(idle, shoot, tune);
  }

  public boolean shooterReady() {
    return currentState().equals("Shoot") && current().isComplete();
  }
}
