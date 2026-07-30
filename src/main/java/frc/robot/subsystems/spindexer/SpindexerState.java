package frc.robot.subsystems.spindexer;

import edu.wpi.first.epilogue.Logged;
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
        new State("Reverse", () -> kicker.reverse().alongWith(spindexer.reverse()))
            .withEnd(() -> !kicker.isStopped() && !spindexer.isStopped()));

    this.kicker = kicker;
    this.spindexer = spindexer;

    /* setup transitions */
    State idle = this.states.get(0);
    State run = this.states.get(1);

    initState(idle);

    idle.to(run).condition(() -> Pilot.shoot().getAsBoolean() && shootState.shooterReady());
    run.to(idle).condition(() -> !Pilot.shoot().getAsBoolean());

    triggersInit();
    transitionsInit();
    configure();
  }

  @Logged
  public boolean spindexerReady() {
    return !kicker.isStopped() && !spindexer.isStopped();
  }
}
