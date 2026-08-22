package frc.robot.subsystems.intake;

import bearlib.statemachine.State;
import bearlib.statemachine.StateMachineBase;
import edu.wpi.first.epilogue.Logged;
import frc.robot.rebuilt.Copilot;
import frc.robot.rebuilt.Pilot;

public class IntakeState extends StateMachineBase {

  public IntakeState(Slider slider, Rollers rollers) {

    State retract =
        new State("Retract", () -> rollers.stop().alongWith(slider.retract()))
            .withEnd(slider::isRetracted);

    State intake =
        new State("Intake", () -> rollers.run().alongWith(slider.extend()))
            .withEnd(() -> slider.isExtended() && !rollers.isStopped());

    State oscillate =
        new State("Oscillate", () -> rollers.runSlow().alongWith(slider.lowOscillate()))
            .withEnd(() -> true);

    State sliderIn = new State("Slider In", () -> slider.retract()).withEnd(slider::isRetracted);

    State sliderOut = new State("Slider Out", () -> slider.extend()).withEnd(slider::isExtended);

    State sliderMid = new State("Slider Mid", () -> slider.mid()).withEnd(() -> true);

    State sliderCalibrate =
        new State("Slider Calibrate", () -> slider.calibrateZero()).withEnd(slider::isCalibrating);

    State sliderIdle = new State("Slider Idle", () -> slider.stop()).withEnd(slider::isStopped);

    State rollersIdle = new State("Rollers Idle", () -> rollers.stop()).withEnd(rollers::isStopped);

    State rollersSlow =
        new State("Rollers Slow", () -> rollers.runSlow()).withEnd(() -> !rollers.isStopped());

    State rollersFast =
        new State("Rollers Fast", () -> rollers.run()).withEnd(() -> !rollers.isStopped());

    State rollersReverseSlow =
        new State("Rollers Reverse Slow", () -> rollers.runSlow())
            .withEnd(() -> !rollers.isStopped());

    State rollersReverse =
        new State("Rollers Reverse", () -> rollers.runSlow()).withEnd(() -> !rollers.isStopped());

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

    configure(
        retract,
        intake,
        oscillate,
        sliderIn,
        sliderOut,
        sliderMid,
        sliderCalibrate,
        sliderIdle,
        rollersIdle,
        rollersSlow,
        rollersFast,
        rollersReverseSlow,
        rollersReverse);
  }

  /** Signals whether the intake is retracting. */
  @Logged
  public boolean retracting() {
    return currentState() == "Retract" && !current().isComplete();
  }
}
