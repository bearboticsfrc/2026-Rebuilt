package frc.robot.subsystems.spindexer;

import bearlib.statemachine.State;
import bearlib.statemachine.StateMachineBase;
import edu.wpi.first.epilogue.Logged;
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

  /** Signals if the spindexer is running. */
  @Logged
  public boolean spindexerReady() {
    return currentState() == "Run" && current().isComplete();
  }
}
