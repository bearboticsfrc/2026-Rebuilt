package frc.robot.subsystems.shooter;

import bearlib.statemachine.State;
import bearlib.statemachine.StateMachineBase;
import edu.wpi.first.epilogue.Logged;
import frc.robot.rebuilt.Copilot;
import frc.robot.rebuilt.Pilot;

public class ShootState extends StateMachineBase {

  private final Flywheel flywheel;

  public ShootState(Flywheel flywheel, Hood hood, DynamicShootingCalculator calculator) {

    super(
        new State("Idle", () -> flywheel.stopCommand().alongWith(hood.stopCommand()))
            .withEnd(() -> true),
        new State(
                "Shoot",
                () ->
                    flywheel
                        .runAtSpeed(() -> calculator.getParameters().flywheelVelocity())
                        .alongWith(
                            hood.goToSetpointRotationsDouble(
                                () -> calculator.getParameters().hoodAngle())))
            .withEnd(flywheel::isAtTarget),
        // Override states
        new State("Flywheel Idle", flywheel::stopCommand).withEnd(flywheel::isStopped),
        new State("Flywheel 500", () -> flywheel.runAtSpeed(500)),
        new State("Flywheel 1.2k", () -> flywheel.runAtSpeed(1200)),
        new State("Flywheel 3.7k", () -> flywheel.runAtSpeed(3700)));

    this.flywheel = flywheel;

    /* setup transitions */
    State idle = this.states.get(0);
    State shoot = this.states.get(1);
    State flywheelIdle = this.states.get(2);
    State flywheel500 = this.states.get(3);
    State flywheel1200 = this.states.get(4);
    State flywheel3700 = this.states.get(5);

    initState(idle);

    idle.to(shoot).condition(Pilot.shoot()::getAsBoolean);

    shoot.to(idle).condition(Pilot.shoot().negate()::getAsBoolean);

    flywheelIdle.global().condition(Copilot.flywheelIdle()::getAsBoolean);

    flywheel500.global().condition(Copilot.flywheel1200()::getAsBoolean);

    flywheel1200.global().condition(Copilot.flywheel1200()::getAsBoolean);

    flywheel3700.global().condition(Copilot.flywheel3700()::getAsBoolean);

    configure();
  }

  /** Signals if the shooter is ready. */
  @Logged
  public boolean shooterReady() {
    return flywheel.isAtTarget();
  }
}
