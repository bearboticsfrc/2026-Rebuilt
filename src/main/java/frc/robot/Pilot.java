package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Pilot {

  private static final CommandXboxController pilot = new CommandXboxController(0);
  private static final RobotState robotState = RobotState.getInstance();

  public static Trigger shoot() {
    Trigger shoot =
        new Trigger(() -> pilot.rightTrigger().getAsBoolean() && !robotState.shootBlocked());
    return shoot;
  }

  public static Trigger intake() {
    return pilot.leftTrigger();
  }

  public static Trigger oscillate() {
    Trigger oscillate =
        new Trigger(() -> pilot.leftBumper().getAsBoolean() && robotState.isStopped());
    return oscillate;
  }

  public static double getLeftX() {
    return -pilot.getLeftX();
  }

  public static double getLeftY() {
    return -pilot.getLeftY();
  }

  public static double getRightX() {
    return -pilot.getRightX();
  }
}
