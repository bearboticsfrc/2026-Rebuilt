package frc.robot.statemachine;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotState;
import frc.robot.subsystems.intake.Rollers;

public class StateMachineTest extends StateMachineBase {

  private static RobotState robotState = RobotState.getInstance();

  public StateMachineTest(Rollers rollers, CommandXboxController pilot) {

    // Construct new StateMachineBase by passing in states.
    super(
        new State("A", () -> rollers.run()).withEnd(() -> rollers.getVelocityInRPM() >= 5000),
        new State("B", () -> rollers.stop()).withEnd(() -> rollers.getVelocityInRPM() == -11110.0));

    // Declare states as objects.
    State a = this.states.get(0);
    State b = this.states.get(1);

    initState(a);

    // Declare state transitions
    a.to(b).condition(() -> robotState.isInNeutralZone().getAsBoolean());
    b.to(a).condition(() -> robotState.isInAllianceZone());

    // Intialize state logic into StateMachineBase.
    triggersInit();
    transitionsInit();
    configure();
  }

  @Logged
  public boolean aIsComplete() {
    return this.get("A").isComplete();
  }

  @Logged
  public boolean bIsComplete() {
    return this.get("B").isComplete();
  }
}
