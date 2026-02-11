package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.spectrumLib.CachedDouble;

public class Intake extends SubsystemBase {
  private final DutyCycleOut m_openLoopRequest = new DutyCycleOut(0);
  private final CachedDouble cachedVoltage;
  private final double retractedPos = 0.0; // Replace with actual retracted position
  private final double extendedPos = 1.0; // Replace with actual extended position
  private final PositionVoltage m_positionRequest = new PositionVoltage(0);
  private final TalonFX flywheelMotor;
  private final TalonFX armMotor;

  public Intake() {
    cachedVoltage = new CachedDouble(this::getVoltage);
    flywheelMotor = new TalonFX(1); // Replace 1 with actual CAN ID
    armMotor = new TalonFX(2); // Replace 2 with actual CAN ID
    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configs.Slot0.kP = 2.0; // Replace 2.0 with actual kP value
    configs.Slot0.kI = 0;
    configs.Slot0.kD = 0.1;

    armMotor.getConfigurator().apply(configs);
  }

  public void runIntake() {
    flywheelMotor.setControl(m_openLoopRequest.withOutput(0.4));
  }

  public void stopIntake() {
    flywheelMotor.setControl(m_openLoopRequest.withOutput(0.0));
  }

  public void extenderOut() {
    double currentPosition = armMotor.getPosition().getValueAsDouble();
    if (currentPosition < extendedPos) {
      armMotor.setControl(m_positionRequest.withPosition(extendedPos));

    } else {
      armMotor.setControl(m_positionRequest.withPosition(currentPosition));
    }
  }

  public void extenderIn() {
    double currentPosition = armMotor.getPosition().getValueAsDouble();
    if (currentPosition > retractedPos) {
      armMotor.setControl(m_positionRequest.withPosition(retractedPos));

    } else {
      armMotor.setControl(m_positionRequest.withPosition(currentPosition));
    }
  }

  public void initSendable(NTSendableBuilder builder) {
    builder.addDoubleProperty("setPoint", this::getSetpointRotations, null);
    builder.addDoubleProperty("Velocity RPM", this::getVelocityRPM, null);
  }

  @Logged
  public double getVoltage() {
    return cachedVoltage.getAsDouble();
  }

  @Logged
  public double getSetpointRotations() {
    return armMotor.getPosition().getValueAsDouble();
  }

  @Logged
  public double getVelocityRPM() {
    return flywheelMotor.getVelocity().getValueAsDouble() * 60; // Convert to RPM
  }
}
