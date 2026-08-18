package frc.robot.statemachine;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import lombok.Getter;

/** A defined state for subsytems/motors on the robot. */
public class State {

  protected String name;
  protected Supplier<Command> action;

  private BooleanSupplier end;

  /**
   * @param name The name of the state.
   * @param action The robot action executed by the state.
   */
  public State(String name, Supplier<Command> action) {
    this.name = name;
    this.action = action;
    this.end = () -> true;
  }

  @Getter ArrayList<Transition> transitions = new ArrayList<>();

  /**
   * Used to assign goal state in a {@link Transition}, followed by {@code .condition()} to define a
   * full {@link Transition}.
   *
   * @param goal The goal state reached in the {@link Transition}
   */
  public Transition to(State goal) {
    return new Transition(this, goal);
  }

  public Transition global() {
    Transition transition = new Transition(this, this);
    transition.global = true;
    return transition;
  }

  /**
   * Assigns an explicit end to a state.
   *
   * @param condition The end condtion, when the state is completed.
   */
  public State withEnd(BooleanSupplier condition) {
    this.end = condition;
    return this;
  }

  /** Signals whether or not a state is completed. */
  public boolean isComplete() {
    return this.end.getAsBoolean();
  }
}
