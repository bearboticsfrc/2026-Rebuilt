package frc.robot.subsystems.spindexer;

import bearlib.statemachine.State;
import bearlib.statemachine.StateMachineBase;
import edu.wpi.first.epilogue.Logged;
import frc.robot.rebuilt.Copilot;
import frc.robot.rebuilt.Pilot;
import frc.robot.subsystems.shooter.ShootState;

public class SpindexerState extends StateMachineBase {

  private final Kicker kicker;
  private final Spindexer spindexer;

  public SpindexerState(Kicker kicker, Spindexer spindexer, ShootState shootState) {

    super(
        new State("Idle", () -> kicker.stop().alongWith(spindexer.stop()))
            .withEnd(() -> kicker.isStopped() && spindexer.isStopped()),
        new State("Run", () -> kicker.run().alongWith(spindexer.run()))
            .withEnd(() -> !kicker.isStopped() && !spindexer.isStopped()),
        // Override states
        new State("Spindex Slow", () -> spindexer.runSlow()).withEnd(() -> !spindexer.isStopped()),
        new State("Spindex Rev Slow", () -> spindexer.reverseSlow())
            .withEnd(() -> !spindexer.isStopped()),
        new State("Spindex Rev", () -> spindexer.reverse()).withEnd(() -> !spindexer.isStopped()));
        new State("Kicker Idle", () -> kicker.stop()).withEnd(() -> kicker.isStopped());
        new State("Kicker Run Slow", () -> kicker.runSlow()).withEnd(() -> !kicker.isStopped());
        new State("Kicker Run", () -> kicker.run()).withEnd(() -> !kicker.isStopped());
        new State("Kicker Rev Slow", () -> kicker.reverseSlow()).withEnd(() -> !kicker.isStopped());
        new State("Kicker Rev", () -> kicker.reverse()).withEnd(() -> !kicker.isStopped());



    /* setup transitions */
    State idle = this.states.get(0);
    State run = this.states.get(1);
    State spindexerRunSlow = this.states.get(2);
    State spindexerReverseSlow = this.states.get(3);
    State spindexerReverse = this.states.get(4);
    State kickerIdle = this.states.get(5);
    State kickerRunSlow = this.states.get(6);
    State kickerRunFast = this.states.get(7);
    State kickerReverseSlow = this.states.get(8);
    State kickerReverse = this.states.get(9);

  

    initState(idle);

    idle.to(run).condition(() -> Pilot.shoot().getAsBoolean() && shootState.shooterReady());

    run.to(idle).condition(Pilot.shoot().negate()::getAsBoolean);

    spindexerRunSlow.global().condition(Copilot.spindexerFwdSlow()::getAsBoolean);

    spindexerReverseSlow.global().condition(Pilot.reverse());

    spindexerReverse.global().condition(Copilot.spindexerRevFast()::getAsBoolean);

    kickerIdle.to(run).condition(() -> Pilot.shoot().getAsBoolean() && shootState.shooterReady());

    kickerRunSlow.global().condition(Copilot.kickerFwdSlow()::getAsBoolean);

    kickerRunFast.global().condition(Copilot.kickerFwdFast()::getAsBoolean);

    kickerReverseSlow.global().condition(Copilot.kickerRevSlow()::getAsBoolean);

    kickerReverse.global().condition(Copilot.kickerRevFast()::getAsBoolean);

    this.kicker = kicker;
    this.spindexer = spindexer;

    configure();
  }

  /** Signals if the spindexer is running. */
  @Logged
  public boolean spindexerReady() {
    return !kicker.isStopped() && !spindexer.isStopped();
  }
}
