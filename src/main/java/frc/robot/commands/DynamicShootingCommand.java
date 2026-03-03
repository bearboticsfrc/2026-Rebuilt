package frc.robot.commands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.trajectory.ShotCalculator;

public class DynamicShootingCommand extends Command {

  private final Hood hood;
  private final Flywheel flywheel;
  private final Spindexer spindexer;
  private final ShotCalculator shotCalculator;

  @Logged public double[] shotCalculations;
  @Logged public double flywheelRPM;
  @Logged public double hoodAngle;

  public DynamicShootingCommand(
      Hood hood, Flywheel flywheel, Spindexer spindexer, CommandSwerveDrivetrain drivetrain) {
    this.hood = hood;
    this.flywheel = flywheel;
    this.spindexer = spindexer;
    shotCalculator =
        new ShotCalculator(
            () -> RobotState.getInstance().turretPose, () -> drivetrain.getChassisSpeeds());
  }

  public Command shoot() {
    return Commands.runOnce(() -> shotCalculator.ShootOnMoveSolver("Hub"))
        .andThen(
            new LogShootParamsCommand(shotCalculator::getFlywheelRPM, shotCalculator::getHoodAngle))
        .andThen(
            flywheel
                .runAtSpeed(shotCalculator::getFlywheelRPM)
                .alongWith(hood.goToSetpointAngle(shotCalculator::getHoodAngle))
                .alongWith(
                    Commands.waitUntil(() -> flywheel.isAtTarget())
                        .andThen(spindexer.runSpindexer())
                        .andThen(spindexer.runTower())));
  }

  public Command stop() {
    return flywheel.stopCommand().andThen(spindexer.stopMotorsCommand());
  }
}
