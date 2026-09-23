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

    State idle = new State("Idle", () -> hood.stopCommand());

    State track =
        new State(
            "Track",
            () -> hood.goToSetpointRotationsDouble(() -> calculator.getParameters().hoodAngle()));

    State ground = new State("Ground", () -> hood.ground());

    State hood100 = new State("Hood 1", () -> hood.goToSetpointRotationsDouble(() -> 1.0));

    idle.to(ground).condition(robotState.decapitateZone()::getAsBoolean);

    track.to(ground).condition(robotState.decapitateZone()::getAsBoolean);

    idle.to(track).condition(Pilot.shoot()::getAsBoolean);

    idle.to(hood100).condition(Copilot.hood1()::getAsBoolean);

    ground.to(idle).condition(() -> !robotState.decapitateZone().getAsBoolean());

    initState(idle);

    configure(idle, track, ground, hood100);
  }

  /** Signals if the hood is trying not to be decapitated. */
  @Logged
  public boolean decapitate() {
    return currentState() == "Ground";
  }
}
