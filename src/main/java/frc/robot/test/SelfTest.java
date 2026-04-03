package frc.robot.test;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.Rollers;
import frc.robot.subsystems.intake.Slider;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.spindexer.Kicker;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;

public class SelfTest {

  private final Map<String, SelfTestable> registry = new LinkedHashMap<>();

  private final Trigger inTestMode = new Trigger(DriverStation::isTest);

  private final Rollers rollers;
  private final Flywheel flywheel;
  private final Hood hood;
  private final Spindexer spindexer;
  private final Kicker kicker;
  private final Turret turret;
  private final Slider slider;
  private final Climber climber;
  private final CommandSwerveDrivetrain drivetrain;

  public SelfTest(
      Rollers rollers,
      Flywheel flywheel,
      Hood hood,
      Spindexer spindexer,
      Kicker kicker,
      Turret turret,
      Slider slider,
      Climber climber,
      CommandSwerveDrivetrain drivetrain) {
    this.rollers = rollers;
    this.flywheel = flywheel;
    this.hood = hood;
    this.spindexer = spindexer;
    this.kicker = kicker;
    this.turret = turret;
    this.slider = slider;
    this.climber = climber;
    this.drivetrain = drivetrain;

    registry.put("rollers", rollers);
    registry.put("flywheel", flywheel);
    registry.put("hood", hood);
    registry.put("spindexer", spindexer);
    registry.put("kicker", kicker);

    registry.put("turret", turret);
    registry.put("slider", slider);
    registry.put("climber", climber);

    // Groups
    TestGroup shooter = new TestGroup("shooter", turret, hood, flywheel);
    TestGroup indexer = new TestGroup("indexer", spindexer, kicker);
    TestGroup intake = new TestGroup("intake", slider, rollers);

    registry.put("shooter", shooter);
    registry.put("indexer", indexer);
    registry.put("intake", intake);

    // Full suite — groups compose recursively
    // registry.put("all", new TestGroup("all", shooter, intake, climber, drivetrain));
  }

  public void bindTriggers() {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();

    StringEntry testGroupEntry = nt.getStringTopic("Robot/Commands/testGroup").getEntry("none");
    StringEntry testSpeedEntry = nt.getStringTopic("Robot/Commands/testSpeed").getEntry("slow");
    BooleanEntry runTestEntry = nt.getBooleanTopic("Robot/Commands/runTest").getEntry(false);

    // publish defaults so topics appear in AdvantageScope
    // testGroupEntry.set("none");
    // testSpeedEntry.set("slow");
    // runTestEntry.set(false);

    new Trigger(runTestEntry::get)
        .and(inTestMode)
        .onTrue(
            Commands.defer(
                () -> {
                  String group = testGroupEntry.get();
                  String speed = testSpeedEntry.get();
                  SelfTestable testable = registry.get(group);

                  if (testable == null) {
                    return Commands.print("SelfTest: unknown group '" + group + "'")
                        .finallyDo(() -> runTestEntry.set(false));
                  }
                  return speed.equals("fast")
                      ? testable.selfTestFast().finallyDo(() -> runTestEntry.set(false))
                      : testable
                          .selfTestSlow()
                          .finallyDo(
                              () -> {
                                System.out.println("setting test entry to false.");
                                runTestEntry.set(false);
                              });
                },
                Set.of()));
  }
}
