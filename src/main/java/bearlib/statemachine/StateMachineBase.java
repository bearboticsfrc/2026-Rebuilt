package bearlib.statemachine;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;

public class StateMachineBase extends SubsystemBase {

  private List<State> states = new ArrayList<>();
  private List<Transition> transitions = new ArrayList<>();
  private HashMap<State, Trigger> stateTriggers = new HashMap<>();

  private State current;
  protected State previous;
  private State initial;

  /**
   * Class for FRC state machine, manages states and their transitions, utilizing the {@link State}
   * and {@link Transition} classes.
   */
  public StateMachineBase() {}

  @Override
  public void periodic() {
    update();
  }

  /**
   * Initializes states, transitions, and actions in proper order. Call this last on init!
   *
   * @param
   */
  public void configure(State... robotStates) {

    Collections.addAll(this.states, robotStates);

    triggersInit();
    transitionsInit();
    actionsInit();

    System.out.println(getName() + " Initialized!");
  }

  /** Manages and monitors transitions from state to state. */
  private void update() {
    for (Transition transition : transitions) {

      if (transition.global()) {

        if (transition.transitionCondition.getAsBoolean()) {
          previous = current;
          current = transition.goal;
        }
      }

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

  /** Initializes every {@link Transition} for every {@link State} in state machine. */
  private void transitionsInit() {
    for (State state : this.states) {
      this.transitions.addAll(state.getTransitions());

      try {
        for (Transition transition : state.transitions) {
          if (transition.global()) {
            Transition undo =
                new Transition(transition.goal, initial)
                    .condition(() -> !transition.transitionCondition.getAsBoolean());
            this.transitions.add(undo);
          }
        }
      } catch (Exception e) {
      }
    }
  }

  /** Creates an "on enter" {@link Trigger} for every state. */
  private void triggersInit() {
    for (State state : this.states) {
      final State s = state;
      this.stateTriggers.put(state, new Trigger(() -> s == current));
    }
  }

  /** Manages proper actions when states are entered. */
  private void actionsInit() {
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
    current = state;
    initial = state;
  }

  /**
   * Trigger used to run action during a specific state.
   *
   * @param state The {@link State} monitored
   */
  public Trigger on(State state) {
    return this.stateTriggers.get(state);
  }

  /** The current state. */
  public State current() {
    return current;
  }

  /** The previous state. */
  public State previous() {
    return previous;
  }

  /** The initial/default state. */
  public State initial() {
    return initial;
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
    return current != null && !current.isComplete() ? current.name : "";
  }
}
