package frc.robot.bearlib.statemachine;

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
 * w/out complicated commands via transition and state logic. For autonmous, use setState() in order
 * to seperate teleop and auto logic.
 *
 * @see {@link State}, {@link Transition}
 */
public class StateMachineBase extends SubsystemBase {

  public StateMachineBase() {}

  /** All the States for the Subsytem. */
  private List<State> states = new ArrayList<>();

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

  /**
   * Initializes states, transitions, and actions in proper order. Must be called only once, and
   * after initState().
   */
  public void configure(State... robotStates) {

    // ensure initState() was called before configure().
    if (current == null) {
      throw new IllegalStateException(getName() + ": call initState() before configure()");
    }

    // add states to stored list of states.
    Collections.addAll(this.states, robotStates);

    // populate state name map.
    for (State state : this.states) {
      if (stateNames.put(state.name(), state) != null) {
        throw new IllegalArgumentException(getName() + ": duplicate state name " + state.name());
      }
    }

    // initialize triggers and actions.
    triggersInit();
    actionsInit();

    System.out.println(getName() + " Initialized!");
  }

  /** Manages and monitors transitions from state to state. */
  protected void update() {
    // monitor available transistions out of current state.
    for (int i = 0; i < current.transitions.size(); i++) {
      Transition transition = current.transitions.get(i);
      // if the transition condition is true, move to the goal state.
      if (transition.transitionCondition.getAsBoolean()) {
        previous = current;
        current = transition.goal;
        return;
      }
    }
  }

  /** Creates a "while active" {@link Trigger} for every state. */
  private void triggersInit() {
    // create a trigger for each state, that is true when current = state.
    for (State state : this.states) {
      final State s = state;
      this.stateTriggers.put(state, new Trigger(() -> s == current));
    }
  }

  /** Manages proper actions when states are entered. */
  private void actionsInit() {
    for (State state : this.states) {

      // get the action for the state, and throw if it fails.
      Command execute;
      try {
        execute = state.action.get();
      } catch (RuntimeException e) {
        throw new IllegalStateException(
            getName() + ": action factory for state '" + state.name + "' threw", e);
      }
      this.on(state)
          .whileTrue(Commands.defer(state.action, execute.getRequirements()).withName(state.name));
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
    // return the trigger for the state, or a false trigger if the state is not found.
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

  /** The current state */
  public String currentState() {
    if (current() != null) {
      return !current().isComplete() ? "Transitioning" : current().name;
    }
    return "Waiting for Init...";
  }

  /** The state that is requested (ie currently being transitioned into). */
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
   * Sets the current state.
   *
   * @param name The state you want to change to.
   */
  public Command setState(String name) {
    State target = getStateByString(name);
    if (target == null) {
      throw new IllegalArgumentException(getName() + ": unknown state '" + name + "'");
    }
    return Commands.runOnce(
        () -> {
          previous = current;
          current = target;
        });
  }

  /** Resets to inital state. */
  public void reset() {
    this.current = initial;
  }
}
