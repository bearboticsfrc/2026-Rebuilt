package frc.robot.statemachine;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.ArrayList;
import lombok.Getter;

/** A defined state for subsytems/motors on the robot. */
public class State {

  protected String name;
  protected Command action;

  public State(String name, Command action) {
    this.name = name;
    this.action = action.withName(name + " state");
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
