package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RobotContainer {

  public void robotInit() {}

  public void teleopInit() {}

  public void autonomousInit() {}

  public void disabledPeriodic() {}

  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}
