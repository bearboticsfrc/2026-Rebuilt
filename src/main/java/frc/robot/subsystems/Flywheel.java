package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.spectrumLib.CachedDouble;

public class Flywheel extends SubsystemBase {
    
  private final CachedDouble cachedVoltage;
  private final TalonFX flywheelMotor;
  private final VelocityDutyCycle velocityOut = new VelocityDutyCycle(0);

  public Flywheel() {
    cachedVoltage = new CachedDouble(this::getVoltage);
    flywheelMotor = new TalonFX(1); // Replace 1 with actual CAN ID
  }

  public Command spin(double targetRPS) {
    return Commands.run(() -> flywheelMotor.setControl(velocityOut.withVelocity(targetRPS)), this);
  }

  @Logged
  public double getVoltage() {
    return cachedVoltage.getAsDouble();
  }
}
