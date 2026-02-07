import subsystems.TurretController;
import subsystems.TurretYAMS;

public class RobotContainer {
     public final TurretYAMS turret = new TurretYAMS(() -> drivetrain.getPose());

  public final TurretController turretController =
      new TurretController(() -> drivetrain.getPose(), value -> turret.setAngle(value));

      drivetrain.registerTelemetry(logger::telemeterize);
    addTurretTestBindings();
  }

  private void addTurretTestBindings() {
    joystick.a().onTrue(turret.setAngle(Degrees.of(180)));
    joystick.b().onTrue(turret.setAngle(Degrees.of(-90)));
    joystick.x().onTrue(turret.setAngle(Degrees.of(90)));
    joystick.y().onTrue(turret.setAngle(Degrees.of(0)));
    joystick.rightBumper().whileTrue(turret.setAngle(() -> turret.turretRelativeRotation()));
  }

}
