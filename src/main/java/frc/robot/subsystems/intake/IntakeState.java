package frc.robot.subsystems.intake;

import edu.wpi.first.epilogue.Logged;
import frc.robot.Pilot;
import frc.robot.statemachine.State;
import frc.robot.statemachine.StateMachineBase;
import java.util.function.BooleanSupplier;

public class IntakeState extends StateMachineBase {

  private final Slider slider;

  public IntakeState(Slider slider, Rollers rollers) {
    super(
        new State("Idle", () -> slider.stop().alongWith(rollers.stop()))
            .withEnd(() -> slider.isStopped() && rollers.isStopped()),
        new State("Retract", () -> rollers.stop().alongWith(slider.retract()))
            .withEnd(slider::isRetracted),
        new State("Extend", () -> rollers.stop().alongWith(slider.extend()))
            .withEnd(slider::isExtended),
        new State("Intake", () -> rollers.run().alongWith(slider.extend()))
            .withEnd(() -> slider.isExtended() && !rollers.isStopped()),
        new State("Oscillate", () -> rollers.runSlow().alongWith(slider.highOscillate()))
            .withEnd(() -> true),
        new State(
                "ManageStall",
                () -> {
                  return slider.manageStall();
                })
            .withEnd(() -> !slider.isStalled()));

    this.slider = slider;

    /* setup transitions */
    State retract = this.states.get(1);
    State intake = this.states.get(3);
    State oscillate = this.states.get(4);
    State manageStall = this.states.get(5);

    initState(retract);

    retract.to(intake).condition(Pilot.intake()::getAsBoolean);

    intake.to(retract).condition(Pilot.intake().negate()::getAsBoolean);

    retract.to(oscillate).condition(Pilot.oscillate()::getAsBoolean);

    oscillate.to(retract).condition(Pilot.oscillate().negate()::getAsBoolean);

    intake.to(oscillate).condition(Pilot.oscillate()::getAsBoolean);

    retract.to(manageStall).condition(slider::isStalled).fallback();

    manageStall.to(retract).condition(() -> true);

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

  @Logged
  public boolean stalled() {
    return this.slider.isStalled();
  }
}
