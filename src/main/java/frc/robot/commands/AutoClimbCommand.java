package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoClimbCommand extends Command {

  private static final Time DRIVE_LEFT_DURATION = Seconds.of(0.05);
  private static final LinearVelocity DRIVE_LEFT_VELOCITY = MetersPerSecond.of(0.5);

  private static final Time DRIVE_BACKWARDS_DURATION = Seconds.of(0.05);
  private static final LinearVelocity DRIVE_BACKWARDS_VELOCITY = MetersPerSecond.of(0.5);

  private final CommandSwerveDrivetrain drivetrain;
  private final Climber climber;

  public AutoClimbCommand(CommandSwerveDrivetrain drivetrain, Climber climber) {
    this.drivetrain = drivetrain;
    this.climber = climber;
  }

  public Command climb() {
    return climber.raise().andThen(driveSideways()).andThen(climber.lower());
  }

  private Command driveSideways() {
    // Use RobotCentric request for simple backwards movement
    SwerveRequest.RobotCentric driveLeft =
        new SwerveRequest.RobotCentric()
            .withVelocityY(DRIVE_LEFT_VELOCITY.in(MetersPerSecond)); // Postive Y is Left
    SwerveRequest.RobotCentric driveBackwards =
        new SwerveRequest.RobotCentric()
            .withVelocityX(
                -DRIVE_BACKWARDS_VELOCITY.in(MetersPerSecond)); // Negative X is backwards

    return drivetrain
        .runOnce(() -> drivetrain.setControl(driveLeft))
        .until(() -> !climber.climberBlocked())
        .withTimeout(.05)
        .andThen(drivetrain.runOnce(() -> drivetrain.setControl(new SwerveRequest.Idle())))
        .andThen(drivetrain.runOnce(() -> drivetrain.setControl(driveBackwards)))
        .andThen(Commands.waitTime(DRIVE_BACKWARDS_DURATION))
        .andThen(drivetrain.runOnce(() -> drivetrain.setControl(new SwerveRequest.Idle()))); // Stop
  }
}
