package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spindexer extends SubsystemBase {

  private final CANBus canivore = new CANBus("Default Name");

  private final TalonFX spindexer = new TalonFX(7, canivore);
  private final TalonFX tower = new TalonFX(23, canivore);

  private final VoltageOut m_voltReq = new VoltageOut(0.0);

  public Spindexer() {

    TalonFXConfiguration spindexerConfig = new TalonFXConfiguration();
    TalonFXConfiguration towerConfig = new TalonFXConfiguration();

    spindexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    spindexerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    spindexerConfig.Slot0.kS = 0;
    spindexerConfig.Slot0.kV = 0;
    spindexerConfig.Slot0.kA = 0;
    spindexerConfig.Slot0.kP = 0;

    towerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    towerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    towerConfig.Slot0.kS = 0;
    towerConfig.Slot0.kV = 0;
    towerConfig.Slot0.kA = 0;
    towerConfig.Slot0.kP = 0;

    for (int i = 0; i < 2; ++i) {
      var status = spindexer.getConfigurator().apply(spindexerConfig);
      if (status.isOK()) break;
    }

    for (int i = 0; i < 2; ++i) {
      var status = tower.getConfigurator().apply(towerConfig);
      if (status.isOK()) break;
    }
  }

  public void setSpindexerOutput(double output) {
    spindexer.setControl(m_voltReq.withOutput(output));
  }

  public void setTowerOutput(double output) {
    tower.setControl(m_voltReq.withOutput(output));
  }

  public void stopMotors() {
    tower.stopMotor();
    spindexer.stopMotor();
  }

  public Command runSpindexer() {
    return runOnce(() -> setSpindexerOutput(0));
  }

  public Command runTower() {
    return runOnce(() -> setTowerOutput(0));
  }

  public Command index() {
    return runOnce(() -> runSpindexer().alongWith(runTower()));
  }

  public Command stopMotorsCommand() {
    return runOnce(() -> stopMotors());
  }

  @Logged
  public double getSpindexerVelocityInRPM() {
    return spindexer.getVelocity().getValue().in(RPM);
  }

  @Logged
  public AngularVelocity getSpindexerVelocity() {
    // Get the current velocity of the spindexer
    return spindexer.getVelocity().getValue();
  }

  @Logged
  public double getTowerVelocityInRPM() {
    return tower.getVelocity().getValue().in(RPM);
  }

  @Logged
  public AngularVelocity getTowerVelocity() {
    // Get the current velocity of the spindexer
    return tower.getVelocity().getValue();
  }
}
