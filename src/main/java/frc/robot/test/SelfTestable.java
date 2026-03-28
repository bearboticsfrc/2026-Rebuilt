package frc.robot.test;

import edu.wpi.first.wpilibj2.command.Command;

public interface SelfTestable {

  // Conservative test -- lower speeds/range
  Command selfTestSlow();

  // Full speed test at normal operating range
  Command selfTestFast();
}
