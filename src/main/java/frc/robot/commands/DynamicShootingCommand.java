package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.trajectory.ShotCalculator;

public class DynamicShootingCommand extends Command {

  private final Hood hood;
  private final Flywheel flywheel;
  private final Spindexer spindexer;
  private final Intake intake;
  private final ShotCalculator shotCalculator;

  @Logged public double[] shotCalculations;
  @Logged public double flywheelRPM;
  @Logged public double hoodAngle;

  public DynamicShootingCommand(
      Hood hood,
      Flywheel flywheel,
      Spindexer spindexer,
      Intake intake,
      CommandSwerveDrivetrain drivetrain) {
    this.hood = hood;
    this.flywheel = flywheel;
    this.spindexer = spindexer;
    this.intake = intake;
    shotCalculator =
        new ShotCalculator(
            () -> RobotState.getInstance().turretPose, () -> drivetrain.getChassisSpeeds());
  }

  public Command shoot() {

    double[] shotCalculations = shotCalculator.ShootOnMoveSolver("Hub");

    double flywheelRPM = shotCalculations[1];
    double hoodAngle = (shotCalculations[2] - 32) / 37.5 * 1.4;

    return flywheel
        .runAtSpeed(flywheelRPM)
        .alongWith(hood.goToSetpointAngle(() -> Rotations.of(hoodAngle)))
        .alongWith(
            Commands.waitUntil(() -> flywheel.isAtTarget())
                .andThen(spindexer.runSpindexer())
                .andThen(spindexer.runTower())
                .andThen(intake.runMouthSlow()));
  }

  public Command stop() {
    return flywheel
        .stopCommand()
        .andThen(spindexer.stopMotorsCommand())
        .andThen(intake.stopMouthCommand());
  }
}
