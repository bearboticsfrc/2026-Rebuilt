package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

    // arm config
    TalonFXConfiguration armConfig = new TalonFXConfiguration();

    armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    armConfig.Slot0.kP = 2.0; // Replace 2.0 with actual kP value
    armConfig.Slot0.kI = 0;
    armConfig.Slot0.kD = 0.1;

    // flywheel config
    TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();

    flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    flywheelConfig.Slot0.kP = 2.0; // Replace 2.0 with actual kP value
    flywheelConfig.Slot0.kI = 0;
    flywheelConfig.Slot0.kD = 0.1;

    // apply configs
    armMotor.getConfigurator().apply(armConfig);
    flywheelMotor.getConfigurator().apply(flywheelConfig);
  }

  // runs the intake
  public Command runIntake() {
    return Commands.run(() -> flywheelMotor.setControl(m_openLoopRequest.withOutput(0.4)), this);
  }

  // stops intake
  public Command stopIntake() {
    return Commands.run(() -> flywheelMotor.setControl(m_openLoopRequest.withOutput(0.0)), this);
  }

  // runs intake in reverse
  public Command reverseIntake() {
    return Commands.run(() -> flywheelMotor.setControl(m_openLoopRequest.withOutput(-0.4)), this);
  }

  // intake out
  public Command extenderOut() {
    double currentPosition = armMotor.getPosition().getValueAsDouble();

    return (currentPosition < extendedPos)
        ? Commands.run(() -> armMotor.setControl(m_positionRequest.withPosition(extendedPos)))
        : Commands.run(() -> armMotor.setControl(m_positionRequest.withPosition(currentPosition)));
  }

  // retract intake
  public Command extenderIn() {
    double currentPosition = armMotor.getPosition().getValueAsDouble();

    return (currentPosition < extendedPos)
        ? Commands.run(() -> armMotor.setControl(m_positionRequest.withPosition(retractedPos)))
        : Commands.run(() -> armMotor.setControl(m_positionRequest.withPosition(currentPosition)));
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

  // activate
  public Command activate() {
    return Commands.run(
        () -> {
          extenderOut().schedule();
          runIntake().schedule();
        });
  }

  // deactivate
  public Command deactivate() {
    return Commands.run(
        () -> {
          stopIntake().schedule();
          extenderIn().schedule();
        });
  }

  // extend run intake
  public Command extendRun() {
    return Commands.run(
        () -> {
          extenderOut().schedule();
          runIntake().schedule();
        });
  }

  // retract stop intake
  public Command retractStop() {
    return Commands.run(
        () -> {
          stopIntake().schedule();
          extenderIn().schedule();
        });
  }
  
  // run reverse
  public Command extendReverse() {
    return Commands.run(
        () -> {
          reverseIntake().schedule();
          extenderOut().schedule();
        });
  }

}
