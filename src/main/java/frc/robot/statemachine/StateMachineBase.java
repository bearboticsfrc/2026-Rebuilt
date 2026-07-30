package frc.robot.statemachine;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;

public class StateMachineBase extends SubsystemBase {

  protected List<State> states = new ArrayList<>();
  protected ArrayList<Transition> transitions = new ArrayList<>();
  protected HashMap<State, Trigger> stateTriggers = new HashMap<>();

  protected State current;
  protected State previous;

  /**
   * Class for FRC state machine, manages states and their transitions, utilizing the {@link State}
   * and {@link Transition} classes.
   *
   * <blockquote>
   *
   * <b>Notice:</b> States (w/ transitions) must be defined before they are passed in.
   *
   * @param states The robot {@link State}(s)
   */
  public StateMachineBase(State... states) {
    this.current = null;
    Collections.addAll(this.states, states);
    previous = current;

    System.out.println(getName() + " Initialized!");
  }

  @Override
  public void periodic() {
    update();
  }

  /** Manages and monitors transitions from state to state. */
  private void update() {
    for (Transition transition : transitions) {
      if (current == transition.origin) {
        if (transition.transitionCondition.getAsBoolean()) {
          current = transition.goal;
          previous = transition.origin;
          return;
        }
        // allow for transition back to a previous state if the current state cannot be completed.
        if (transition.goal == previous
            && transition.transitionRequest.getAsBoolean()
            && !current.isComplete()) {
          current = transition.goal;
          previous = transition.origin;
        }
      }
    }
  }

  /**
   * Trigger used to run action during a specific state.
   *
   * @param state The {@link State} monitored
   */
  public Trigger on(State state) {
    return this.stateTriggers.get(state);
  }

  /** Logs the current state */
  @Logged
  public String currentState() {
    if (current != null) {
      return !current.isComplete() ? "Transitioning" : current.name;
    }
    return "Waiting for Init...";
  }

  /** Logs the state that is requested (ie currently being transitioned into). */
  @Logged
  public String requested() {
    return !current.isComplete() ? current.name : "";
  }

  /**
   * Initializes every {@link Transition} for every {@link State} in state machine. Call this on
   * initialization!
   */
  public void transitionsInit() {
    for (State state : this.states) {
      this.transitions.addAll(state.getTransitions());
    }
  }

  /** Creates an "on enter" {@link Trigger} for every state. Call this on initialization! */
  protected void triggersInit() {
    for (State state : this.states) {
      final State s = state;
      this.stateTriggers.put(state, new Trigger(() -> s == current));
    }
  }

  /** Manages proper actions when states are entered. Call this last on initialization! */
  protected void configure() {
    for (State state : this.states) {
      this.on(state)
          .onTrue(
              Commands.defer(state.action, state.action.get().getRequirements())
                  .withName(getName() + "." + state.name));
    }
  }

  /**
   * Sets the starting state of the state machine.
   *
   * @param state The starting state.
   */
  protected void initState(State state) {
    if (!states.isEmpty() && states.contains(state)) {
      current = state;
    }
  }

  /**
   * Returns the corresponding state via the state's name.
   *
   * @param name The name of the state.
   */
  protected State get(String name) {
    for (State state : this.states) {
      if (name.equals(state.name)) return state;
    }
    return null;
  }
}
