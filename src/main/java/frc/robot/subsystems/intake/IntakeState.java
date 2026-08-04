package frc.robot.subsystems.intake;

import bearlib.statemachine.State;
import bearlib.statemachine.StateMachineBase;
import edu.wpi.first.epilogue.Logged;
import frc.robot.rebuilt.Copilot;
import frc.robot.rebuilt.Pilot;
import java.util.function.BooleanSupplier;

public class IntakeState extends StateMachineBase {

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
        new State("Oscillate", () -> rollers.runSlow().alongWith(slider.lowOscillate()))
            .withEnd(() -> true),

        // Override states
        new State("Slider In", () -> slider.retract()).withEnd(slider::isRetracted),
        new State("Slider Out", () -> slider.extend()).withEnd(slider::isExtended),
        new State("Slider Mid", () -> slider.mid()).withEnd(() -> true),
        new State("Slider Calibrate", () -> slider.calibrateZero()).withEnd(slider::isCalibrating),
        new State("Slider Idle", () -> slider.stop()).withEnd(slider::isStopped),
        new State("Rollers Idle", () -> rollers.stop()).withEnd(rollers::isStopped),
        new State("Rollers Slow", () -> rollers.runSlow()).withEnd(() -> !rollers.isStopped()),
        new State("Rollers Fast", () -> rollers.run()).withEnd(() -> !rollers.isStopped()),
        new State("Rollers Reverse Slow", () -> rollers.runSlow())
            .withEnd(() -> !rollers.isStopped()),
        new State("Rollers Reverse", () -> rollers.runSlow()).withEnd(() -> !rollers.isStopped()));

    /* setup transitions */
    State retract = this.states.get(1);
    State intake = this.states.get(3);
    State oscillate = this.states.get(4);
    State sliderIn = this.states.get(5);
    State sliderOut = this.states.get(6);
    State sliderCalibrate = this.states.get(8);
    State sliderMid = this.states.get(7);
    State sliderIdle = this.states.get(9);
    State rollersIdle = this.states.get(10);
    State rollersSlow = this.states.get(11);
    State rollersFast = this.states.get(12);
    State rollersReverseSlow = this.states.get(13);
    State rollersReverse = this.states.get(14);

    initState(retract);

    retract.to(intake).condition(Pilot.intake()::getAsBoolean);

    intake.to(retract).condition(Pilot.intake().negate()::getAsBoolean);

    retract.to(oscillate).condition(Pilot.oscillate()::getAsBoolean);

    oscillate.to(retract).condition(Pilot.oscillate().negate()::getAsBoolean);

    intake.to(oscillate).condition(Pilot.oscillate()::getAsBoolean);

    sliderIn.global().condition(Copilot.sliderIn()::getAsBoolean);

    sliderOut.global().condition(Copilot.sliderOut()::getAsBoolean);

    sliderCalibrate.global().condition(Copilot.sliderCalibrate()::getAsBoolean);

    sliderMid.global().condition(Copilot.sliderMiddle()::getAsBoolean);

    sliderIdle.global().condition(Copilot.sliderIdle()::getAsBoolean);

    rollersIdle.global().condition(Copilot.rollerIdle()::getAsBoolean);

    rollersSlow.global().condition(Copilot.rollerFwdSlow()::getAsBoolean);

    rollersFast.global().condition(Copilot.rollerFwdFast()::getAsBoolean);

    rollersReverseSlow.global().condition(Copilot.rollerRevSlow()::getAsBoolean);

    rollersReverse.global().condition(Copilot.rollerRevFast()::getAsBoolean);

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
}
