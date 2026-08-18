package frc.robot.statemachine;

import java.util.function.BooleanSupplier;

/**
 * A defined change in a state machine, from one {@link State} to another, under a specific
 * condition.
 *
 * <blockquote>
 *
 * <b>Notice:</b> For a proper functioning state machine, the {@code
 * origin.to(goal).condition(condition)}usage must be used
 */
public class Transition {

  public final State origin;
  public final State goal;
  public BooleanSupplier transitionCondition;
  public BooleanSupplier transitionRequest;
  public boolean global;

  /**
   * Default Constructor for a Transition, not used in implementation.
   *
   * @param origin The {@link State} of origin.
   * @param goal The goal {@link State} to move to.
   * @param condition The condition required to move from origin to goal.
   */
  private Transition(State origin, State goal, BooleanSupplier condition) {
    this.origin = origin;
    this.goal = goal;
    this.transitionCondition = condition;
    this.transitionRequest = condition;
    this.global = false;
  }

  /**
   * Constructor, allows for a cleaner approach to define state transitions via{@code
   * origin.to(goal).condition(condition)}. Not used in implementation.
   *
   * @param origin The {@link State} of origin.
   * @param end The goal {@link State} to move to.
   */
  public Transition(State origin, State goal) {
    this(origin, goal, () -> true);
  }

  /**
   * Used for a clean approach to add a {@link Transition} to a {@link State}.
   *
   * <blockquote>
   *
   * <b>Notice:</b> For a state transition to "automatically" occur/occur when an origin state is
   * complete, pass in a {@link BooleanSupplier} that returns true.
   *
   * @param condition The condition required.
   */
  public Transition condition(BooleanSupplier condition) {
    this.transitionRequest = condition;

    // only use transition request if transition is forced
    if (this.global) {
      this.transitionCondition = () -> condition.getAsBoolean();
      this.origin.transitions.add(this);
    }
    // normal behavior
    else {
      this.transitionCondition = () -> condition.getAsBoolean() && this.origin.isComplete();
      this.origin.transitions.add(this);
    }
    return this;
  }

  /**
   * Added to a transition to override saftey logic. Intended use onto emergency states, where a
   * forced transition would not cause mechanical damage
   */
  public Transition fallback() {
    this.origin.transitions.remove(this);
    this.transitionCondition = transitionRequest;
    this.origin.transitions.add(this);
    return this;
  }

  /** Signals whether or not a transition is global, meaning it can be entered from any state. */
  public boolean global() {
    return this.global;
  }
}
