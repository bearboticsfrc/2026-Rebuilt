package frc.robot.state;

import bot.den.foxflow.RobotState;
import bot.den.foxflow.StateMachine;

/**
 * Composite robot state combining driver-station mode, intake state, and shooter state.
 *
 * <p>{@link RobotState} is managed automatically by FoxFlow based on the driver station control
 * word — it cannot be set via the constructor or {@code transitionTo()}. {@link IntakeState} and
 * {@link ShooterState} are transitioned by driver inputs and sensor conditions.
 *
 * <p>FoxFlow generates {@code GameStateStateMachine} from this record at compile time.
 */
@StateMachine
public record GameState(
    RobotState robotState, IntakeState intakeState, ShooterState shooterState) {}
