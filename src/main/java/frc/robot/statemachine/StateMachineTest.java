package frc.robot.statemachine;

import frc.robot.Robot;
import frc.robot.RobotState;

public class StateMachineTest extends StateMachineBase {

  private static RobotState robotState = RobotState.getInstance();

  public static State A = new State("A", Robot.get().getFlywheel().runAtSpeed(0));
  public static State B = new State("B", Robot.get().getFlywheel().runAtSpeed(500));

  private static StateMachineTest instance;

  public static StateMachineTest getInstance() {
    if (instance == null) instance = new StateMachineTest();
    return instance;
  }

  static {
    A.to(B).condition(() -> robotState.isInNeutralZone());
    B.to(A).condition(() -> robotState.isInAllianceZone());
  }

  public StateMachineTest() {
    super(A, B);
  }

  @Override
  public void periodic() {
    super.periodic();
  }
}
