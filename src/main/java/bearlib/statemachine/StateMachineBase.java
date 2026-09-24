package bearlib.statemachine;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;

/**
 * Base class for state machine, for a singular subsystem or mechanism. States are a representations
 * of simple actions, associated with simple commands. More complex behavior can be cleanly managed
 * w/out complicated commands via transition and state logic.
 *
 * @see {@link State}, {@link Transition}
 */
public class StateMachineBase extends SubsystemBase {

  public StateMachineBase() {}

  /** All the States for the Subsytem. */
  private List<State> states = new ArrayList<>();

  /** All the defined transistions in states */
  private List<Transition> transitions = new ArrayList<>();

  /** Map for all the triggers for when current = a specific state. */
  private HashMap<State, Trigger> stateTriggers = new HashMap<>();

  /* Map to quickly get a state via name lookup.*/
  private HashMap<String, State> stateNames = new HashMap<>();

  /** The current, "running" state. */
  private State current;

  /** The previous, "exited" state. */
  protected State previous;

  /** The initial state. Used as default. */
  private State initial;

  @Override
  public void periodic() {
    // Limiting update allows forcing states w/ setState() during auto.
    if (!RobotState.isAutonomous()) {
      update();
    }
  }

  /** Initializes states, transitions, and actions in proper order. Call this last on init! */
  public void configure(State... robotStates) {

    // add states to stored list of states.
    Collections.addAll(this.states, robotStates);

    // populate state name map.
    for (State state : this.states) {
      stateNames.put(state.name(), state);
    }

    triggersInit();
    transitionsInit();
    actionsInit();

    System.out.println(getName() + " Initialized!");
  }

  /** Manages and monitors transitions from state to state. */
  protected void update() {

    // saftey precaution, and stay in state w/out exit.
    if (current == null || current.transitions == null) {
      return;
    }

    for (Transition transition : current.transitions) {
      // If transistion can occur and is requested.
      if (transition.transitionCondition.getAsBoolean()) {
        current = transition.goal;
        previous = transition.origin;
        return;
      }
    }
  }

  /** Initializes every {@link Transition} for every {@link State} in state machine. */
  private void transitionsInit() {
    for (State state : this.states) {
      this.transitions.addAll(state.getTransitions());
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
          .whileTrue(
              Commands.defer(state.action, state.action.get().getRequirements())
                  .withName(state.name));
    }
  }

  /**
   * Sets the starting state of the state machine. Cannot be global.
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
    return this.stateTriggers.getOrDefault(state, Trigger.kFalse);
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
    if (current() != null) {
      return !current().isComplete() ? "Transitioning" : current().name;
    }
    return "Waiting for Init...";
  }

  /** Logs the state that is requested (ie currently being transitioned into). */
  @Logged
  public String requested() {
    return current() != null && !current.isComplete() ? current().name : "";
  }

  /**
   * Returns a state through its corresponding name.
   *
   * @param state The name of the state.
   */
  public State getStateByString(String state) {
    return stateNames.get(state);
  }

  /**
   * Sets the current state, only functions during autonomous.
   *
   * @param state The state you want to change to.
   */
  public Command setState(String state) {
    return Commands.runOnce(() -> this.current = this.getStateByString(state));
  }

  /** Resets to inital state. */
  public void reset() {
    this.current = initial;
  }
}
