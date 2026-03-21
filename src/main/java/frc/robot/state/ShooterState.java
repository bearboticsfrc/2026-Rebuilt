package frc.robot.state;

import bot.den.foxflow.LimitsStateTransitions;
import bot.den.foxflow.StateMachine;

/**
 * Tracks the shooter mechanism's phase.
 *
 * <p>The flywheel, hood, and spindexer are commanded by transitions between these states. The
 * SPINNING_UP → SHOOTING boundary is crossed automatically once the flywheel reaches target
 * velocity.
 */
@StateMachine
public enum ShooterState implements LimitsStateTransitions<ShooterState> {
  /** Flywheel stopped, hood idle. */
  IDLE,

  /** Flywheel and hood moving to target parameters; spindexer not yet running. */
  SPINNING_UP,

  /** Flywheel at target speed — spindexer is feeding and the note is in flight. */
  SHOOTING;

  @Override
  public boolean canTransitionState(ShooterState newState) {
    return switch (this) {
      case IDLE -> newState == SPINNING_UP;
      case SPINNING_UP -> newState == SHOOTING || newState == IDLE;
      case SHOOTING -> newState == IDLE;
    };
  }
}
