package frc.robot.subsystems.shooter;

import bearlib.statemachine.State;
import bearlib.statemachine.StateMachineBase;
import edu.wpi.first.epilogue.Logged;
import frc.robot.RobotState;
import frc.robot.rebuilt.Copilot;
import frc.robot.rebuilt.Pilot;

public class ShootState extends StateMachineBase {

  DynamicShootingCalculator calculator = DynamicShootingCalculator.getInstance();

  RobotState robotState = RobotState.getInstance();

  public ShootState(Flywheel flywheel, Hood hood) {

    State idle =
        new State("Idle", () -> flywheel.runAtSpeed(600).alongWith(hood.stopCommand()))
            .withEnd(() -> true);

    State shoot =
        new State(
                "Shoot",
                () ->
                    (flywheel
                        .runAtSpeed(() -> calculator.getParameters().flywheelVelocity())
                        .alongWith(
                            hood.goToSetpointRotationsDouble(
                                () -> calculator.getParameters().hoodAngle()))))
            .withEnd(() -> true);

    State flywheelIdle =
        new State("Flywheel Idle", () -> flywheel.stopCommand()).withEnd(flywheel::isStopped);

    State flywheel500 =
        new State("Flywheel 500", () -> flywheel.runAtSpeed(500)).withEnd(flywheel::isAtTarget);

    State flywheel1200 =
        new State("Flywheel 1.2k", () -> flywheel.runAtSpeed(1200)).withEnd(flywheel::isAtTarget);

    State flywheel3700 =
        new State("Flywheel 3.7k", () -> flywheel.runAtSpeed(3700)).withEnd(flywheel::isAtTarget);

    State hoodIdle = new State("Hood Idle", () -> hood.stopCommand()).withEnd(() -> true);

    State hood25 =
        new State("Hood .25", () -> hood.goToSetpointRotationsDouble(() -> 0.25))
            .withEnd(() -> true);

    State hood50 =
        new State("Hood .5", () -> hood.goToSetpointRotationsDouble(() -> 0.5)).withEnd(() -> true);

    State hood75 =
        new State("Hood .75", () -> hood.goToSetpointRotationsDouble(() -> 0.75))
            .withEnd(() -> true);

    State hood100 =
        new State("Hood 1", () -> hood.goToSetpointRotationsDouble(() -> 1.0)).withEnd(() -> true);

    State hoodGround =
        new State("Hood Ground", () -> hood.ground().alongWith(flywheel.runAtSpeed(600)))
            .withEnd(() -> true);

    idle.to(shoot).condition(Pilot.shoot()::getAsBoolean);

    shoot.to(idle).condition(Pilot.shoot().negate()::getAsBoolean);

    flywheelIdle.global().condition(Copilot.flywheelIdle()::getAsBoolean);

    flywheel500.global().condition(Copilot.flywheel1200()::getAsBoolean);

    flywheel1200.global().condition(Copilot.flywheel1200()::getAsBoolean);

    flywheel3700.global().condition(Copilot.flywheel3700()::getAsBoolean);

    hood75.global().condition(Copilot.hood0_75()::getAsBoolean);

    hood25.global().condition(Copilot.hood0_25()::getAsBoolean);

    hoodIdle.global().condition(Copilot.hoodIdle()::getAsBoolean);

    hood100.global().condition(Copilot.hood1()::getAsBoolean);

    hoodGround.global().condition(robotState::decapitateZone);

    initState(idle);

    configure(
        idle,
        shoot,
        flywheelIdle,
        flywheel500,
        flywheel1200,
        flywheel3700,
        hoodIdle,
        hood25,
        hood50,
        hood75,
        hood100,
        hoodGround);
  }

  /** Signals if the shooter is ready. */
  @Logged
  public boolean shooterReady() {
    return currentState() == "Shoot" && current().isComplete();
  }

  @Logged
  public boolean decapitation() {
    return currentState() == "Hood Ground";
  }
}
