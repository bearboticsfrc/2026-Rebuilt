package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Class used for cleaner pilot controller reference and abstraction. */
public class Pilot {

  private static final CommandXboxController pilot =
      new CommandXboxController(0); // driver controller
  private static final RobotState robotState = RobotState.getInstance();

  /** Returns a trigger to shoot based on driver input and field restrictions. */
  public static Trigger shoot() {
    Trigger shoot =
        new Trigger(() -> pilot.rightTrigger().getAsBoolean() && !robotState.shootBlocked());
    return shoot;
  }

  /** Returns a trigger to intake based on driver input. */
  public static Trigger intake() {
    return pilot.leftTrigger();
  }

  /** Returns a trigger to intake based on driver input. */
  public static Trigger oscillate() {
    Trigger oscillate =
        new Trigger(() -> pilot.leftBumper().getAsBoolean() && robotState.isStopped());
    return oscillate;
  }

  /** Returns value of driver input on the left x-axis. */
  public static double getLeftX() {
    return -pilot.getLeftX();
  }

  /** Returns value of driver input on the left y-axis. */
  public static double getLeftY() {
    return -pilot.getLeftY();
  }

  /** Returns value of driver input on the right x-axis. */
  public static double getRightX() {
    return -pilot.getRightX();
  }
}
