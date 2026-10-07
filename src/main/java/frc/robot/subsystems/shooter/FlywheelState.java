package frc.robot.subsystems.shooter;

import frc.robot.RobotState;
import frc.robot.bearlib.statemachine.State;
import frc.robot.bearlib.statemachine.StateMachineBase;
import frc.robot.bearlib.util.TunableNumber;
import frc.robot.rebuilt.Pilot;

public class FlywheelState extends StateMachineBase {

  private final Flywheel flywheel;

  DynamicShootingCalculator calculator = DynamicShootingCalculator.getInstance();

  RobotState robotState = RobotState.getInstance();

  public FlywheelState(Flywheel flywheel, TunableNumber rpm) {

    this.flywheel = flywheel;

    State idle = new State("Idle", () -> flywheel.runAtSpeed(0.0));

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

  public boolean shooterReady() {
    return flywheel.isAtTarget();
  }
}
