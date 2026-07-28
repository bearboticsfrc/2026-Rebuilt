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

  private List<State> states = new ArrayList<>();
  private ArrayList<Transition> transitions = new ArrayList<>();
  private HashMap<State, Trigger> stateTriggers = new HashMap<>();

  protected State current;

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
    Collections.addAll(this.states, states);
    this.current = this.states.get(0);

    transitionsInit();
    populateStateTriggers();
    execute();
  }

  @Override
  public void periodic() {
    update();
  }

  /** Manages and monitors transitions from state to state. */
  private void update() {
    for (Transition transition : transitions) {
      if (current == transition.origin) {
        if (transition.condition.get()) {
          current = transition.goal;
          return;
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
  @Logged(name = "Current State")
  public String currentState() {
    return current.name;
  }

  /** Initializes every {@link Transition} for every {@link State} in state machine. */
  public void transitionsInit() {
    for (State state : this.states) {
      this.transitions.addAll(state.getTransitions());
    }
  }

  /** Creates an "on enter" {@link Trigger} for every state. */
  private void populateStateTriggers() {
    for (State state : this.states) {
      final State s = state;
      this.stateTriggers.put(state, new Trigger(() -> s == current));
    }
  }

  /** Manages and executes proper actions when states are entered. */
  private void execute() {
    for (State state : this.states) {
      this.on(state).onTrue(Commands.runOnce(state.action).finallyDo(()-> state.action));
    }
  }
}
