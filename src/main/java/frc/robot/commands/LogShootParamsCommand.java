package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class LogShootParamsCommand extends InstantCommand {
  /**
   * Creates a new a PrintCommand.
   *
   * @param message the message to print
   */
  public LogShootParamsCommand(Supplier<Double> flywheelRPM, Supplier<Angle> hoodAngle) {
    super(
        () ->
            System.out.println(
                "Dynamic Shoot with flywheel = "
                    + flywheelRPM.get()
                    + " hoodAngle = "
                    + hoodAngle.get().in(Rotations)));
  }

  /**
   * Creates a new a PrintCommand.
   *
   * @param message the message to print
   */
  public LogShootParamsCommand(
      String source, DoubleSupplier flywheelRPM, DoubleSupplier hoodAngle) {
    super(
        () ->
            System.out.println(
                source
                    + " with flywheel = "
                    + flywheelRPM.getAsDouble()
                    + " hoodAngle = "
                    + hoodAngle.getAsDouble()));
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
