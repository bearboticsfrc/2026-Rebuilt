package frc.robot.subsystems;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotState;
import frc.robot.subsystems.intake.Slider;
import frc.robot.subsystems.shooter.Flywheel;
import lombok.Getter;

public class StateMachine extends SubsystemBase {

  public enum States {
    DRIVE,
    INTAKE,
    SHOOTING,
    CLIMB,
  }

  public enum ShootStates {
    IDLE,
    RAMP,
    SHOOT,
  }

  public enum IntakeStates {
    RETRACT,
    EXTENDING,
    EXTENDED,
    OSCILLATE,
  }

  public enum TurretStates {
    TRACK,
    IDLE,
  }

  public enum ClimbStates {
    IDLE,
    EXTENDING,
    EXTENDED,
    RETRACTING,
    RETRACTED,
  }

  @Getter private States state;
  private States previousState;

  @Getter private ShootStates shootState;
  private ShootStates previousShootState;

  @Getter private IntakeStates intakeState;
  private IntakeStates previousIntakeState;

  @Getter private TurretStates turretState;
  private TurretStates previousTurretState;

  @Getter private ClimbStates climbState;
  private ClimbStates previousClimbState;

  private final RobotState robotState = RobotState.getInstance();

  private final CommandXboxController pilot;
  private final CommandPS5Controller copilot;
  private final Flywheel flywheel;
  private final Slider slider;
  private final Climber climber;

  public StateMachine(
      CommandXboxController pilot,
      CommandPS5Controller copilot,
      Flywheel flywheel,
      Slider slider,
      Climber climber) {

    this.pilot = pilot;
    this.copilot = copilot;
    this.flywheel = flywheel;
    this.slider = slider;
    this.climber = climber;

    this.state = States.DRIVE;
    this.previousState = States.DRIVE;

    this.shootState = ShootStates.IDLE;
    this.previousShootState = ShootStates.IDLE;

    this.intakeState = IntakeStates.RETRACT;
    this.previousIntakeState = IntakeStates.RETRACT;

    this.turretState = TurretStates.IDLE;
    this.previousTurretState = TurretStates.TRACK;

    this.climbState = ClimbStates.IDLE;
    this.previousClimbState = ClimbStates.IDLE;
  }

  private void update() {
    switch (state) {
      case DRIVE:
        transition(States.DRIVE, States.INTAKE, pilot.getLeftTriggerAxis() > 0.1);
        transition(
            States.DRIVE,
            States.SHOOTING,
            robotState.isInAllianceZone() && pilot.getRightTriggerAxis() > 0.1);
        transition(
            States.DRIVE, States.CLIMB, climber.isAtBottom() && copilot.povUp().getAsBoolean());
        break;
      case SHOOTING:
        transition(
            States.SHOOTING,
            States.DRIVE,
            pilot.getRightTriggerAxis() < 0.1 || !robotState.isInAllianceZone());
        break;
      case INTAKE:
        transition(States.INTAKE, States.DRIVE, pilot.getLeftTriggerAxis() < 0.1);
        break;
      case CLIMB:
        transition(
            States.CLIMB,
            States.DRIVE,
            !copilot.povUp().getAsBoolean()
                && !copilot.povDown().getAsBoolean()
                && climber.isAtBottom());
        break;
      default:
        transition(States.DRIVE, States.DRIVE, true);
        break;
    }
  }

  private void updateShootState() {
    switch (shootState) {
      case IDLE:
        break;
      case RAMP:
        transition(ShootStates.RAMP, ShootStates.SHOOT, flywheel.isAtTarget());
        break;
      case SHOOT:
        break;
    }
  }

  private void updateIntakeState() {
    switch (intakeState) {
      case RETRACT:
        transition(
            IntakeStates.RETRACT,
            IntakeStates.OSCILLATE,
            robotState.isStopped() && pilot.getRightTriggerAxis() > 0.1);
        break;
      case EXTENDING:
        transition(IntakeStates.EXTENDING, IntakeStates.EXTENDED, slider.isExtended());
        break;
      case EXTENDED:
        transition(
            IntakeStates.EXTENDED,
            IntakeStates.OSCILLATE,
            robotState.isStopped() && pilot.getRightTriggerAxis() > 0.1);
        break;
      case OSCILLATE:
        transition(IntakeStates.OSCILLATE, IntakeStates.RETRACT, pilot.getRightTriggerAxis() < 0.1);
        transition(
            IntakeStates.OSCILLATE,
            IntakeStates.EXTENDING,
            !robotState.isStopped() && pilot.getRightTriggerAxis() > 0.1);
        break;
    }
  }

  private void updateTurretState() {
    switch (turretState) {
      case IDLE:
        transition(TurretStates.IDLE, TurretStates.TRACK, true);
        break;
      case TRACK:
        break;
    }
  }

  private void updateClimbState() {
    switch (climbState) {
      case IDLE:
        break;
      case EXTENDING:
        transition(ClimbStates.EXTENDING, ClimbStates.EXTENDED, climber.isAtTop());
        break;
      case EXTENDED:
        transition(
            ClimbStates.EXTENDED,
            ClimbStates.RETRACTING,
            climber.canClimb() && copilot.povDown().getAsBoolean());
        break;
      case RETRACTING:
        transition(ClimbStates.RETRACTING, ClimbStates.RETRACTED, climber.isAtBottom());
        break;
      case RETRACTED:
        break;
    }
  }

  @Override
  public void periodic() {
    update();
    updateShootState();
    updateIntakeState();
    updateTurretState();
    updateClimbState();
    updatePreviousStates();
  }

  public void transition(States current, States goal, boolean condition) {
    if (state == current && condition) {
      logStateTransition(state, goal);
      state = goal;

      switch (goal) {
        case DRIVE:
          shootState = ShootStates.IDLE;
          intakeState = IntakeStates.RETRACT;
          turretState = TurretStates.TRACK;
          climbState = ClimbStates.IDLE;
          break;
        case INTAKE:
          shootState = ShootStates.IDLE;
          intakeState = IntakeStates.EXTENDING;
          turretState = TurretStates.TRACK;
          climbState = ClimbStates.IDLE;
          break;
        case SHOOTING:
          shootState = ShootStates.RAMP;
          intakeState = IntakeStates.EXTENDING;
          turretState = TurretStates.TRACK;
          climbState = ClimbStates.IDLE;
          break;
        case CLIMB:
          shootState = ShootStates.IDLE;
          intakeState = IntakeStates.RETRACT;
          turretState = TurretStates.TRACK;
          climbState = ClimbStates.EXTENDING;
          break;
      }
    }
  }

  public void transition(ShootStates current, ShootStates goal, boolean condition) {
    if (shootState == current && condition) {
      logStateTransition(shootState, goal);
      shootState = goal;
    }
  }

  public void transition(IntakeStates current, IntakeStates goal, boolean condition) {
    if (intakeState == current && condition) {
      logStateTransition(intakeState, goal);
      intakeState = goal;
    }
  }

  public void transition(TurretStates current, TurretStates goal, boolean condition) {
    if (turretState == current && condition) {
      logStateTransition(turretState, goal);
      turretState = goal;
    }
  }

  public void transition(ClimbStates current, ClimbStates goal, boolean condition) {
    if (climbState == current && condition) {
      logStateTransition(climbState, goal);
      climbState = goal;
    }
  }

  private void updatePreviousStates() {
    previousState = state;
    previousShootState = shootState;
    previousIntakeState = intakeState;
    previousTurretState = turretState;
    previousClimbState = climbState;
  }

  public Trigger onExit(States exit) {
    return new Trigger(() -> previousState == exit && state != exit);
  }

  public Trigger onExit(ShootStates exit) {
    return new Trigger(() -> previousShootState == exit && shootState != exit);
  }

  public Trigger onExit(IntakeStates exit) {
    return new Trigger(() -> previousIntakeState == exit && intakeState != exit);
  }

  public Trigger onExit(TurretStates exit) {
    return new Trigger(() -> previousTurretState == exit && turretState != exit);
  }

  public Trigger onExit(ClimbStates exit) {
    return new Trigger(() -> previousClimbState == exit && climbState != exit);
  }

  public Trigger onEnter(States enter) {
    return new Trigger(() -> state == enter && previousState != enter);
  }

  public Trigger onEnter(ShootStates enter) {
    return new Trigger(() -> shootState == enter && previousShootState != enter);
  }

  public Trigger onEnter(IntakeStates enter) {
    return new Trigger(() -> intakeState == enter && previousIntakeState != enter);
  }

  public Trigger onEnter(TurretStates enter) {
    return new Trigger(() -> turretState == enter && previousTurretState != enter);
  }

  public Trigger onEnter(ClimbStates enter) {
    return new Trigger(() -> climbState == enter && previousClimbState != enter);
  }

  private void logStateTransition(States from, States to) {
    DogLog.log("StateMachine/States ", from + " -> " + to);
  }

  private void logStateTransition(ShootStates from, ShootStates to) {
    DogLog.log("StateMachine/ShootStates ", from + " -> " + to);
  }

  private void logStateTransition(IntakeStates from, IntakeStates to) {
    DogLog.log("StateMachine/IntakeStates ", from + " -> " + to);
  }

  private void logStateTransition(TurretStates from, TurretStates to) {
    DogLog.log("StateMachine/TurretStates ", from + " -> " + to);
  }

  private void logStateTransition(ClimbStates from, ClimbStates to) {
    DogLog.log("StateMachine/ClimbStates ", from + " -> " + to);
  }
}
