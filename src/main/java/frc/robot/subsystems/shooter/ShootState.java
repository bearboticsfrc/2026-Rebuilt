package frc.robot.subsystems.shooter;

import bearlib.statemachine.State;
import bearlib.statemachine.StateMachineBase;
import edu.wpi.first.epilogue.Logged;
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
            .withEnd(flywheel::isAtTarget));

    this.flywheel = flywheel;

    /* setup transitions */
    State idle = this.states.get(0);
    State shoot = this.states.get(1);

    initState(idle);

    idle.to(shoot).condition(Pilot.shoot()::getAsBoolean);

    shoot.to(idle).condition(Pilot.shoot().negate()::getAsBoolean);

    triggersInit();
    transitionsInit();
    configure();
  }

  /** Signals if the shooter is ready. */
  @Logged
  public boolean shooterReady() {
    return flywheel.isAtTarget();
  }
}
