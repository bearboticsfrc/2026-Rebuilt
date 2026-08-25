package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;

import bearlib.statemachine.State;
import bearlib.statemachine.StateMachineBase;
import edu.wpi.first.epilogue.Logged;
import frc.robot.rebuilt.Copilot;

public class TurretState extends StateMachineBase {

  public TurretState(Turret turret) {

    State idle = new State("Idle", turret::stop).withEnd(() -> true);

    State track = new State("Track", turret::getDefaultCommand).withEnd(() -> true);

    State zeroDegrees =
        new State("Zero Degrees", () -> turret.setAngle(Degrees.of(0)))
            .withEnd(() -> turret.isNearTarget(Degrees.of(0)));

    State degrees45 =
        new State("45 Degrees", () -> turret.setAngle(Degrees.of(45)))
            .withEnd(() -> turret.isNearTarget(Degrees.of(45)));

    State degrees90 =
        new State("90 Degrees", () -> turret.setAngle(Degrees.of(90)))
            .withEnd(() -> turret.isNearTarget(Degrees.of(90)));

    State degrees135 =
        new State("135 Degrees", () -> turret.setAngle(Degrees.of(135)))
            .withEnd(() -> turret.isNearTarget(Degrees.of(135)));

    State degrees180 =
        new State("180 Degrees", () -> turret.setAngle(Degrees.of(180)))
            .withEnd(() -> turret.isNearTarget(Degrees.of(180)));

    State degrees225 =
        new State("225 Degrees", () -> turret.setAngle(Degrees.of(225)))
            .withEnd(() -> turret.isNearTarget(Degrees.of(225)));

    State degrees270 =
        new State("270 Degrees", () -> turret.setAngle(Degrees.of(270)))
            .withEnd(() -> turret.isNearTarget(Degrees.of(270)));

    State degrees315 =
        new State("315 Degrees", () -> turret.setAngle(Degrees.of(315)))
            .withEnd(() -> turret.isNearTarget(Degrees.of(315)));

    idle.global().condition(Copilot.turretIdle()::getAsBoolean);

    zeroDegrees.global().condition(Copilot.turret0Degrees()::getAsBoolean);

    degrees45.global().condition(Copilot.turret45Degrees()::getAsBoolean);

    degrees90.global().condition(Copilot.turret90Degrees()::getAsBoolean);

    degrees135.global().condition(Copilot.turret135Degrees()::getAsBoolean);

    degrees180.global().condition(Copilot.turret180Degrees()::getAsBoolean);

    degrees225.global().condition(Copilot.turret225Degrees()::getAsBoolean);

    degrees270.global().condition(Copilot.turret270Degrees()::getAsBoolean);

    degrees315.global().condition(Copilot.turret315Degrees()::getAsBoolean);

    initState(track);

    configure(
        idle,
        track,
        zeroDegrees,
        degrees45,
        degrees90,
        degrees135,
        degrees180,
        degrees225,
        degrees270,
        degrees315);
  }

  /** Signals whether or not the turret is tracking. */
  @Logged
  public boolean tracking() {
    return currentState() == "Track";
  }
}
