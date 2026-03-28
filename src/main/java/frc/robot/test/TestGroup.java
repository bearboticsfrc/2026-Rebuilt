package frc.robot.test;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.List;
import java.util.function.Function;

public class TestGroup implements SelfTestable {

  private final String name;
  private final List<SelfTestable> tests;

  public TestGroup(String name, SelfTestable... tests) {
    this.name = name;
    this.tests = List.of(tests);
  }

  @Override
  public Command selfTestSlow() {
    return sequence(SelfTestable::selfTestSlow).withName(name + ".SelfTestSlow");
  }

  @Override
  public Command selfTestFast() {
    return sequence(SelfTestable::selfTestFast).withName(name + ".SelfTestFast");
  }

  private Command sequence(Function<SelfTestable, Command> fn) {
    return tests.stream().map(fn).reduce(Command::andThen).orElse(Commands.none());
  }
}
