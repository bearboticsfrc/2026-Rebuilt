package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.spectrumLib.CachedDouble;

public class Flywheel extends SubsystemBase {
   
  private final CachedDouble cachedVoltage;
  private final TalonFX flywheelMotor;
  private final VelocityDutyCycle m_velocityOut = new VelocityDutyCycle(0);

  public Flywheel() {
    cachedVoltage = new CachedDouble(this::getVoltage);
    flywheelMotor = new TalonFX(1); // Replace 1 with actual CAN ID

  // motor config
  TalonFXConfiguration spindexMotorConfig = new TalonFXConfiguration();

  // configure the configuration object (do not try to access MotorOutput/Slot0 on the motor instance)
  spindexMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
  spindexMotorConfig.Slot0.kP = 0;
  spindexMotorConfig.Slot0.kI = 0;
  spindexMotorConfig.Slot0.kD = 0;

  // apply the configuration to the TalonFX controller
  flywheelMotor.getConfigurator().apply(spindexMotorConfig);
  }

  public Command runFlywheel(double targetRPS) {
    return Commands.run(() -> flywheelMotor.setControl(m_velocityOut.withVelocity(targetRPS)), this);
  }

  @Logged
  public double getVoltage() {
    return cachedVoltage.getAsDouble();
  }
}
