import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.IntakeState;
import frc.robot.subsystems.shooter.FlywheelState;
import frc.robot.subsystems.shooter.HoodState;
import frc.robot.subsystems.spindexer.SpindexerState;
import frc.robot.subsystems.turret.TurretState;

public class Auto {

  private final SpindexerState spindexerState;
  private final TurretState turretState;
  private final IntakeState intakeState;
  private final FlywheelState flywheelState;
  private final HoodState hoodState;

  private final EventTrigger shoot = new EventTrigger("shoot");
  

  public Auto(
      SpindexerState spindexerState,
      TurretState turretState,
      IntakeState intakeState,
      FlywheelState flywheelState,
      HoodState hoodState) {
    this.spindexerState = spindexerState;
    this.turretState = turretState;
    this.intakeState = intakeState;
    this.flywheelState = flywheelState;
    this.hoodState = hoodState;

    shoot.onTrue(shootCommand());

  }

  public Command shootCommand() {
    return Commands.runOnce(() -> hoodState.setState("Track"))
        .alongWith(Commands.runOnce(() -> flywheelState.setState("Shoot")).alongWith(Commands.run(() -> spindexerState.setState("Run"))));
  }
  

  public Command intakeCommand() {
    return Commands.runOnce(() -> intakeState.setState("Intake")).alongWith(Commands.runOnce(() -> ))
  }

  //public Command stopShootCommand

  //public Command oscillateCommand

}
