package frc.robot.subsystems.shooter;

import bearlib.statemachine.State;
import bearlib.statemachine.StateMachineBase;
import edu.wpi.first.epilogue.Logged;
import frc.robot.RobotState;
import frc.robot.rebuilt.Pilot;

public class FlywheelState extends StateMachineBase {

  DynamicShootingCalculator calculator = DynamicShootingCalculator.getInstance();

  RobotState robotState = RobotState.getInstance();

  public FlywheelState(Flywheel flywheel) {

    State idle = new State("Idle", () -> flywheel.runAtSpeed(601));

    State shoot =
        new State(
                "Shoot",
                () -> flywheel.runAtSpeed(() -> calculator.getParameters().flywheelVelocity()))
            .withEnd(() -> true);

    idle.to(shoot).condition(Pilot.shoot()::getAsBoolean);

    shoot.to(idle).condition(Pilot.shoot().negate()::getAsBoolean);

    initState(idle);

    configure(idle, shoot);
  }

  /** Signals if the shooter is ready. */
  @Logged
  public boolean shooterReady() {
    return currentState() == "Shoot" && current().isComplete();
  }
}
