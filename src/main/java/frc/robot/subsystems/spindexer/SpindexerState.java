package frc.robot.subsystems.spindexer;

import frc.robot.bearlib.statemachine.State;
import frc.robot.bearlib.statemachine.StateMachineBase;
import frc.robot.rebuilt.Pilot;
import frc.robot.subsystems.shooter.FlywheelState;

public class SpindexerState extends StateMachineBase {

  public SpindexerState(Kicker kicker, Spindexer spindexer, FlywheelState flywheelState) {

    State idle = new State("Idle", () -> kicker.stop().alongWith(spindexer.stop()));

    State run = new State("Run", () -> kicker.run().alongWith(spindexer.run()));

    idle.to(run).condition(() -> Pilot.shoot().getAsBoolean() && flywheelState.shooterReady());

    run.to(idle).condition(Pilot.shoot().negate()::getAsBoolean);

    initState(idle);

    configure(idle, run);
  }
}
