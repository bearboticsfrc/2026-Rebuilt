package frc.robot.subsystems.intake;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Pilot;
import frc.robot.statemachine.State;
import frc.robot.statemachine.StateMachineBase;
import java.util.function.BooleanSupplier;

public class IntakeState extends StateMachineBase {

  private final Slider slider;

  private boolean retractStall;
  private boolean tryRetract = false;
  private final Timer timer = new Timer();

  @Override
  public void periodic() {

    manageRetractStall();
    super.periodic();
  }

  public IntakeState(Slider slider, Rollers rollers) {
    super(
        new State("Idle", () -> slider.stop().alongWith(rollers.stop()))
            .withEnd(() -> slider.isStopped() && rollers.isStopped()),
        new State("Retract", () -> rollers.stop().alongWith(slider.retract()))
            .withEnd(() -> slider.isRetracted()),
        new State("Extend", () -> rollers.stop().alongWith(slider.extend()))
            .withEnd(() -> slider.isExtended()),
        new State("Intake", () -> rollers.run().alongWith(slider.extend()))
            .withEnd(() -> slider.isExtended() && !rollers.isStopped()),
        new State("Oscillate", () -> rollers.runSlow().alongWith(slider.highOscillate()))
            .withEnd(() -> true));

    this.slider = slider;

    /* setup transitions */
    State idle = this.states.get(0);
    State retract = this.states.get(1);
    State intake = this.states.get(3);
    State oscillate = this.states.get(4);

    initState(retract);

    retract.to(intake).condition(() -> Pilot.intake().getAsBoolean());
    intake.to(retract).condition(() -> !Pilot.intake().getAsBoolean());
    retract.to(oscillate).condition(() -> Pilot.oscillate().getAsBoolean());
    intake.to(idle).condition(() -> slider.isStalled()).fallback();
    idle.to(intake).condition(() -> Pilot.intake().getAsBoolean());
    retract.to(idle).condition(() -> slider.isStalled() && !slider.isCalibrating()).fallback();
    idle.to(retract).condition(() -> tryRetract);
    oscillate.to(retract).condition(() -> !Pilot.oscillate().getAsBoolean());
    intake.to(oscillate).condition(() -> Pilot.oscillate().getAsBoolean());

    triggersInit();
    transitionsInit();
    configure();
  }

  /**
   * Signals whether or not the retract state is being entered, becomes false when retract state is
   * completed.
   */
  @Logged
  public BooleanSupplier retracting() {
    return () -> this.current == this.get("Retract") && !current.isComplete();
  }

  /** Manages slider stall while retracting in order to reduce screams of terror. */
  private void manageRetractStall() {
    // Start timer when stall occurs while retracting.
    if (retracting().getAsBoolean() && slider.isStalled() && !retractStall) {
      tryRetract = false;
      retractStall = true;
      timer.restart();
    }
    // Check if stall was fixed after 1 second, and try retracting.
    if (retractStall && timer.hasElapsed(1.0)) {
      tryRetract = true;
      timer.stop();
      timer.reset();
      retractStall = false;
    }
    // Don't keep trying retract once retract state is reached
    if (this.current == this.get("Retract")) tryRetract = false;
  }
}
