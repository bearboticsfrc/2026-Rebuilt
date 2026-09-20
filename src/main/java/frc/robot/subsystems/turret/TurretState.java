package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;

import bearlib.statemachine.State;
import bearlib.statemachine.StateMachineBase;
import edu.wpi.first.epilogue.Logged;

public class TurretState extends StateMachineBase {

  public TurretState(Turret turret) {

    State idle = new State("Idle", turret::stop).withEnd(() -> true);

    State track = new State("Track", turret::getDefaultCommand).withEnd(() -> true);

    State zeroDegrees =
        new State("Zero Degrees", () -> turret.setAngle(Degrees.of(0)))
            .withEnd(() -> turret.isNearTarget(Degrees.of(0)));

    initState(track);

    configure(idle, track, zeroDegrees);
  }

  /** Signals whether or not the turret is tracking. */
  @Logged
  public boolean tracking() {
    return currentState() == "Track";
  }
}
