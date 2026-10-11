package frc.robot.subsystems.shooter;

import frc.robot.RobotState;
import frc.robot.bearlib.statemachine.State;
import frc.robot.bearlib.statemachine.StateMachineBase;
import frc.robot.bearlib.util.TunableNumber;
import frc.robot.rebuilt.Copilot;
import frc.robot.rebuilt.Pilot;

public class HoodState extends StateMachineBase {

  DynamicShootingCalculator calculator = DynamicShootingCalculator.getInstance();

  RobotState robotState = RobotState.getInstance();

  public HoodState(Hood hood, TunableNumber rotations) {

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

    track.to(ground).condition(Pilot.shoot().negate()::getAsBoolean);

    idle.to(hood100).condition(Copilot.hood1()::getAsBoolean);

    ground.to(idle).condition(() -> !robotState.decapitateZone().getAsBoolean());

    initState(idle);

    configure(idle, track, ground, hood100);
  }
}
