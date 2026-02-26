package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.trajectory.ShotCalculator;
import frc.robot.subsystems.turret.Turret;

public class ShooterCommand {

  private final Hood hood;
  private final Flywheel flywheel;
  private final Turret turret;
  private final CommandSwerveDrivetrain drivetrain;
  private final ShotCalculator shotCalculator;

  public ShooterCommand(
      Hood hood, Flywheel flywheel, Turret turret, CommandSwerveDrivetrain drivetrain) {
    this.hood = hood;
    this.flywheel = flywheel;
    this.turret = turret;
    this.drivetrain = drivetrain;
    shotCalculator =
        new ShotCalculator(() -> drivetrain.getPose(), () -> drivetrain.getChassisSpeeds());
  }

  public Command shoot() {
    return new InstantCommand(
        () -> {
          double[] shotCalculations =
              shotCalculator.ShootOnMoveSolver(shotCalculator.targetLocation());
          double flywheelRPM = shotCalculations[1];
          hood.goToSetpointAngle(() -> Degrees.of(shotCalculations[2]));
          flywheel.setVelocity(Units.RPM.of(flywheelRPM));
        },
        hood,
        flywheel);
  }
}
