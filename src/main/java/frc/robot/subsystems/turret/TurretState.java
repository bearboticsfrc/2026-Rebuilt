package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import frc.robot.bearlib.statemachine.State;
import frc.robot.bearlib.statemachine.StateMachineBase;
import frc.robot.rebuilt.Copilot;
import frc.robot.subsystems.shooter.DynamicShootingCalculator;

public class TurretState extends StateMachineBase {

  private final DynamicShootingCalculator calculator = DynamicShootingCalculator.getInstance();

  public TurretState(Turret turret) {

    State idle = new State("Idle", turret::stop).withEnd(() -> true);

    State track =
        new State(
                "Track",
                () ->
                    turret.setAngle(
                        () ->
                            Radians.of(
                                calculator.getParameters().turretAngle().getMeasure().in(Radians)),
                        () -> calculator.getParameters().turretVelocity()))
            .withEnd(() -> true);

    State zeroDegrees =
        new State("Zero Degrees", () -> turret.setAngle(Degrees.of(0)))
            .withEnd(() -> turret.isNearTarget(Degrees.of(0)));

    initState(idle);

    idle.to(track).condition(Copilot.turretIdle().negate()::getAsBoolean);

    track.to(idle).condition(Copilot.turretIdle()::getAsBoolean);

    configure(idle, track, zeroDegrees);
  }
}
