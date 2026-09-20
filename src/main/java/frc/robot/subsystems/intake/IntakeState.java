package frc.robot.subsystems.intake;

import bearlib.statemachine.State;
import bearlib.statemachine.StateMachineBase;
import edu.wpi.first.epilogue.Logged;
import frc.robot.rebuilt.Pilot;

public class IntakeState extends StateMachineBase {

  public IntakeState(Slider slider, Rollers rollers) {

    State retract = new State("Retract", () -> rollers.stop().alongWith(slider.retract()));

    State intake = new State("Intake", () -> rollers.run().alongWith(slider.extend()));

    State oscillate =
        new State("Oscillate", () -> rollers.runSlow().alongWith(slider.lowOscillate()));

    initState(retract);

    retract.to(intake).condition(Pilot.intake()::getAsBoolean);

    intake.to(retract).condition(Pilot.intake().negate()::getAsBoolean);

    retract.to(oscillate).condition(Pilot.oscillate()::getAsBoolean);

    oscillate.to(retract).condition(Pilot.oscillate().negate()::getAsBoolean);

    intake.to(oscillate).condition(Pilot.oscillate()::getAsBoolean);

    configure(retract, intake, oscillate);
  }

  @Logged
  public boolean retract() {
    return currentState() == "Retract";
  }
}
