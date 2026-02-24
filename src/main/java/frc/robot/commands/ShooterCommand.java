package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.trajectoryCalculator.ShotCalculator;

public class ShooterCommand {

  private final Hood hood = new Hood();
  private final Flywheel flywheel = new Flywheel();
  private final Turret turret = new Turret();
  // private final CommandSwerveDrivetrain drivetrain = new CommandSwerveDrivetrain();
  private final ShotCalculator shotCalculator =
      new ShotCalculator(drivetrain.getPose(), drivetrain.getChassisSpeeds());

  public ShooterCommand() {}

  public Command shoot() {
    return new InstantCommand(
        () -> {
          double[] shotCalculations =
              shotCalculator.ShootOnMoveSolver(shotCalculator.targetLocation());
          double flywheelRPM = shotCalculations[1];
          hood.goToSetpointAngle(() -> Degrees.of(shotCalculations[2]));
          flywheel.setVelocity(Units.RPM.of(flywheelRPM));
          // turret.setAngle(Degrees.of(shotCalculations[3]));
        },
        hood,
        flywheel /*, turret*/);
  }
}
