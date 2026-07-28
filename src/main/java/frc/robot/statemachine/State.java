package frc.robot.statemachine;

import java.util.ArrayList;
import lombok.Getter;

/** A defined state for subsytems/motors on the robot. */
public class State {

  protected String name;
  protected Runnable action;

  public State(String name, Runnable action) {
    this.name = name;
    this.action = action;
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
}
