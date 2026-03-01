package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spindexer extends SubsystemBase {

  private final CANBus canivore = new CANBus("Default Name");

  private final TalonFX spindexer = new TalonFX(7, canivore);
  private final TalonFX tower = new TalonFX(23, canivore);

  private final DutyCycleOut m_dutyReq = new DutyCycleOut(0.0);
  private final double TOWER_OUTPUT = 1;
  private final double SPINDEXER_OUTPUT = 0.7;

  public Spindexer() {
    super("Spindexer");

    TalonFXConfiguration spindexerConfig = new TalonFXConfiguration();
    TalonFXConfiguration towerConfig = new TalonFXConfiguration();

    spindexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    spindexerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    towerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    towerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    CurrentLimitsConfigs towerCurrentLimitsConfigs = new CurrentLimitsConfigs();
    towerCurrentLimitsConfigs.StatorCurrentLimit = 240;
    towerCurrentLimitsConfigs.SupplyCurrentLimit = 140;
    towerCurrentLimitsConfigs.StatorCurrentLimitEnable = true;
    towerCurrentLimitsConfigs.SupplyCurrentLimitEnable = true;
    towerConfig.withCurrentLimits(towerCurrentLimitsConfigs);

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
    spindexer.setControl(m_dutyReq.withOutput(output));
  }

  public void setTowerOutput(double output) {
    tower.setControl(m_dutyReq.withOutput(output));
  }

  public void stopMotors() {
    tower.stopMotor();
    spindexer.stopMotor();
  }

  public Command runSpindexer() {
    return runOnce(() -> setSpindexerOutput(SPINDEXER_OUTPUT));
  }

  public Command runTower() {
    return runOnce(() -> setTowerOutput(TOWER_OUTPUT));
  }

  public Command index() {
    return runSpindexer().andThen(runTower());
  }

  public Command stopMotorsCommand() {
    return runOnce(() -> stopMotors());
  }

  @Logged
  public double getSpindexerVelocityInRPM() {
    return spindexer.getVelocity().getValue().in(RPM);
  }

  @Logged
  public double getTowerVelocityInRPM() {
    return tower.getVelocity().getValue().in(RPM);
  }
}
