package frc.robot.subsystems.spindexer;

import bearlib.statemachine.State;
import bearlib.statemachine.StateMachineBase;
import edu.wpi.first.epilogue.Logged;
import frc.robot.rebuilt.Copilot;
import frc.robot.rebuilt.Pilot;
import frc.robot.subsystems.shooter.ShootState;

public class SpindexerState extends StateMachineBase {

  public SpindexerState(Kicker kicker, Spindexer spindexer, ShootState shootState) {

    State idle =
        new State("Idle", () -> kicker.stop().alongWith(spindexer.stop()))
            .withEnd(() -> kicker.isStopped() && spindexer.isStopped());

    State run =
        new State("Run", () -> kicker.run().alongWith(spindexer.run()))
            .withEnd(() -> !kicker.isStopped() && !spindexer.isStopped());

    State spindexerRunSlow =
        new State("Spindex Slow", () -> spindexer.runSlow()).withEnd(() -> !spindexer.isStopped());

    State spindexerReverseSlow =
        new State("Spindex Rev Slow", () -> spindexer.reverseSlow())
            .withEnd(() -> !spindexer.isStopped());

    State spindexerReverse =
        new State("Spindex Rev", () -> spindexer.reverse()).withEnd(() -> !spindexer.isStopped());

    State kickerIdle = new State("Kicker Idle", () -> kicker.stop()).withEnd(kicker::isStopped);

    State kickerRunSlow =
        new State("Kicker Run Slow", () -> kicker.runSlow()).withEnd(() -> !kicker.isStopped());

    State kickerRunFast =
        new State("Kicker Run", () -> kicker.run()).withEnd(() -> !kicker.isStopped());

    State kickerReverseSlow =
        new State("Kicker Rev Slow", () -> kicker.reverseSlow()).withEnd(() -> !kicker.isStopped());

    State kickerReverse =
        new State("Kicker Rev", () -> kicker.reverse()).withEnd(() -> !kicker.isStopped());

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

    initState(idle);

    configure(
        idle,
        run,
        spindexerRunSlow,
        spindexerReverseSlow,
        spindexerReverse,
        kickerIdle,
        kickerRunSlow,
        kickerRunFast,
        kickerReverseSlow,
        kickerReverse);
  }

  /** Signals if the spindexer is running. */
  @Logged
  public boolean spindexerReady() {
    return currentState() == "Run" && current().isComplete();
  }
}
