package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.trajectory.ShotCalculator;

public class ShooterCommand {

  private final Hood hood;
  private final Flywheel flywheel;
  private final CommandSwerveDrivetrain drivetrain;
  private final ShotCalculator shotCalculator;

  public ShooterCommand(Hood hood, Flywheel flywheel, CommandSwerveDrivetrain drivetrain) {
    this.hood = hood;
    this.flywheel = flywheel;
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
          double hoodAngle = (shotCalculations[2] - 32) / 37.5 * 1.4;
          hood.goToSetpointAngle(() -> Rotations.of(hoodAngle));
          flywheel.runAtSpeed(flywheelRPM);
        },
        hood,
        flywheel);
  }
}
