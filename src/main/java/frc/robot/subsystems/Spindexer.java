package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.spectrumLib.CachedDouble;

public class Spindexer extends SubsystemBase {
  private final DutyCycleOut m_openLoopRequest = new DutyCycleOut(0);
  private final CachedDouble cachedVoltage;
  private final TalonFX spindexMotor;

  public Spindexer() {
    cachedVoltage = new CachedDouble(this::getVoltage);
    spindexMotor = new TalonFX(1); // add actual CAN ID

    // motor config
    TalonFXConfiguration spindexMotorConfig = new TalonFXConfiguration();

    spindexMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    spindexMotorConfig.Slot0.kP = 0;
    spindexMotorConfig.Slot0.kI = 0;
    spindexMotorConfig.Slot0.kD = 0;
  }

  // runs spindexer
  public Command spindex() {
    return Commands.run(() -> spindexMotor.setControl(m_openLoopRequest.withOutput(1)), this);
  }

  @Logged
  public double getVoltage() {
    return cachedVoltage.getAsDouble();
  }
}
