package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Pilot {

  private static final CommandPS5Controller pilot = new CommandPS5Controller(0);
  private static final RobotState robotState = RobotState.getInstance();

  public static Trigger shoot() {
    return pilot.R2();
  }

  public static Trigger intake() {
    return pilot.L2();
  }

  public static Trigger oscillate() {
    Trigger oscillate = new Trigger(() -> pilot.L1().getAsBoolean() && robotState.isStopped());
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
