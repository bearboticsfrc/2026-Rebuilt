package frc.robot.subsystems.shooter;

import bearlib.statemachine.State;
import bearlib.statemachine.StateMachineBase;
import edu.wpi.first.epilogue.Logged;
import frc.robot.RobotState;
import frc.robot.rebuilt.Copilot;
import frc.robot.rebuilt.Pilot;

public class HoodState extends StateMachineBase {

  DynamicShootingCalculator calculator = DynamicShootingCalculator.getInstance();

  RobotState robotState = RobotState.getInstance();

  public HoodState(Hood hood) {

    State idle = new State("Idle", () -> hood.stopCommand()).withEnd(() -> true);

    State track =
        new State(
            "Track",
            () -> hood.goToSetpointRotationsDouble(() -> calculator.getParameters().hoodAngle()));

    State hood25 =
        new State("Hood .25", () -> hood.goToSetpointRotationsDouble(() -> 0.25))
            .withEnd(() -> true);

    State hood50 =
        new State("Hood .5", () -> hood.goToSetpointRotationsDouble(() -> 0.5)).withEnd(() -> true);

    State hood75 =
        new State("Hood .75", () -> hood.goToSetpointRotationsDouble(() -> 0.75))
            .withEnd(() -> true);

    State hood100 =
        new State("Hood 1", () -> hood.goToSetpointRotationsDouble(() -> 1.0)).withEnd(() -> true);

    State ground = new State("Ground", () -> hood.ground()).withEnd(() -> true);

    idle.to(track).condition(Pilot.shoot()::getAsBoolean);

    track.to(idle).condition(Pilot.shoot().negate()::getAsBoolean);

    hood75.global().condition(Copilot.hood0_75()::getAsBoolean);

    hood25.global().condition(Copilot.hood0_25()::getAsBoolean);

    hood50.global().condition(Copilot.hood0_5()::getAsBoolean);

    idle.global().condition(Copilot.hoodIdle()::getAsBoolean);

    hood100.global().condition(Copilot.hood1()::getAsBoolean);

    ground.global().condition(robotState::decapitateZone);

    initState(idle);

    configure(idle, track, hood75, hood25, hood50, hood100, ground);
  }

  /** Signals if the hood is trying not to be decapitated. */
  @Logged
  public boolean decapitate() {
    return currentState() == "Ground";
  }
}
