package frc.robot.subsystems.spindexer;

import edu.wpi.first.epilogue.Logged;
import frc.robot.Copilot;
import frc.robot.Pilot;
import frc.robot.statemachine.State;
import frc.robot.statemachine.StateMachineBase;
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
        new State("Spindexer Run Slow", () -> spindexer.run())
            .withEnd(() -> !spindexer.isStopped());
        new State("Spindexer Reverse Slow", () -> spindexer.reverseSlow())
            .withEnd(() -> !spindexer.isStopped());
        new State("Spindexer Reverse", () -> spindexer.reverse())
            .withEnd(() -> !spindexer.isStopped()));
  
    this.kicker = kicker;
    this.spindexer = spindexer;

    /* setup transitions */
    State idle = this.states.get(0);
    State run = this.states.get(1);
    State spindexerRunSlow = this.states.get(2);
    State spindexerReverseSlow = this.states.get(3);
    State spindexerReverse = this.states.get(4);

    initState(idle);

    idle.to(run).condition(() -> Pilot.shoot().getAsBoolean() && shootState.shooterReady());

    run.to(idle).condition(Pilot.shoot().negate()::getAsBoolean);

    spindexerRunSlow.global().condition(Pilot.shoot().negate()::getAsBoolean);

    spindexerReverseSlow.global().condition(Copilot.spindexerRevSlow().negate()::getAsBoolean);

    spindexerReverse.global().condition(Copilot.spindexerRevFast().negate()::getAsBoolean);

    triggersInit();
    transitionsInit();
    configure();
  }

  /** Signals if the spindexer is running. */
  @Logged
  public boolean spindexerReady() {
    return !kicker.isStopped() && !spindexer.isStopped();
  }
}
