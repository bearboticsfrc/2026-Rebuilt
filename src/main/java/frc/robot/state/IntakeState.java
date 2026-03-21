package frc.robot.state;

import bot.den.foxflow.LimitsStateTransitions;
import bot.den.foxflow.StateMachine;

/**
 * Tracks whether the intake mechanism is active.
 *
 * <p>The arm's physical position and roller speed are driven by commands scheduled on state
 * transitions — this enum represents the logical intent, not the mechanical position.
 */
@StateMachine
public enum IntakeState implements LimitsStateTransitions<IntakeState> {
  /** Arm retracted, roller stopped. */
  IDLE,

  /** Arm extending or extended, roller spinning to accept a game piece. */
  INTAKING;

  @Override
  public boolean canTransitionState(IntakeState newState) {
    return switch (this) {
      case IDLE -> newState == INTAKING;
      case INTAKING -> newState == IDLE;
    };
  }
}
