package frc.robot.statemachine;

import java.util.function.Supplier;

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
  public Supplier<Boolean> condition;

  /**
   * Default Constructor for a Transition, not used in implementation.
   *
   * @param origin The {@link State} of origin.
   * @param goal The goal {@link State} to move to.
   * @param condition The condition required to move from origin to goal.
   */
  private Transition(State origin, State goal, Supplier<Boolean> condition) {
    this.origin = origin;
    this.goal = goal;
    this.condition = condition;
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
   * @param condition The condition required
   */
  public Transition condition(Supplier<Boolean> condition) {
    this.condition = condition;
    this.origin.transitions.add(this);
    return this;
  }
}
