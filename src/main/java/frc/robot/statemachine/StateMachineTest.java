package frc.robot.statemachine;

public class StateMachineTest extends StateMachine {

  public static State A = new State("A");
  public static State B = new State("B");
  private int counter = 0;
  private static boolean secondsPassed = false;

  private static StateMachineTest instance;

  public static StateMachineTest getInstance() {
    if (instance == null) instance = new StateMachineTest();
    return instance;
  }

  static {
    A.to(B).condition(() -> secondsPassed);
  }

  public StateMachineTest() {
    super(A, B);
  }

  @Override
  public void periodic() {
    super.periodic();

    if (counter++ > 500) {
      secondsPassed = true;
    }
  }
}
