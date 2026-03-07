package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
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
  private final VelocityTorqueCurrentFOC m_vtcfReq = new VelocityTorqueCurrentFOC(0.0);
  private final double TOWER_OUTPUT = 1;
  private final double TOWER_REVERSE_OUTPUT = -0.2;

  private final double SPINDEXER_OUTPUT = 0.7;
  private final double SPINDEXER_REVERSE_OUTPUT = -0.2;

  public Spindexer() {
    super("Spindexer");

    TalonFXConfiguration spindexerConfig = new TalonFXConfiguration();
    TalonFXConfiguration towerConfig = new TalonFXConfiguration();

    spindexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    spindexerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    CurrentLimitsConfigs spindexerCurrentLimitsConfigs = new CurrentLimitsConfigs();

    spindexerCurrentLimitsConfigs.StatorCurrentLimit = 240;
    spindexerCurrentLimitsConfigs.SupplyCurrentLimit = 140;
    spindexerCurrentLimitsConfigs.StatorCurrentLimitEnable = true;
    spindexerCurrentLimitsConfigs.SupplyCurrentLimitEnable = true;

    towerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    towerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    CurrentLimitsConfigs towerCurrentLimitsConfigs = new CurrentLimitsConfigs();
    towerCurrentLimitsConfigs.StatorCurrentLimit = 240;
    towerCurrentLimitsConfigs.SupplyCurrentLimit = 140;
    towerCurrentLimitsConfigs.StatorCurrentLimitEnable = true;
    towerCurrentLimitsConfigs.SupplyCurrentLimitEnable = true;
    towerConfig.withCurrentLimits(towerCurrentLimitsConfigs);

    StatusCode status = spindexer.getConfigurator().apply(spindexerConfig);

    for (int i = 0; i < 2; ++i) {
      if (status.isOK()) break;
      status = spindexer.getConfigurator().apply(spindexerConfig);
    }
    if (!status.isOK()) {
      System.out.println("ERROR Configuring Spindexer spindexer motor: " + status);
    }

    status = tower.getConfigurator().apply(towerConfig);
    for (int i = 0; i < 2; ++i) {
      if (status.isOK()) break;
      status = tower.getConfigurator().apply(towerConfig);
    }
    if (!status.isOK()) {
      System.out.println("ERROR Configuring Spindexer tower motor: " + status);
    }

    optimizeCAN();
  }

  private void optimizeCAN() {
    spindexer.getPosition().setUpdateFrequency(50);
    spindexer.getVelocity().setUpdateFrequency(50);
    spindexer.getSupplyCurrent().setUpdateFrequency(50);
    spindexer.getDeviceTemp().setUpdateFrequency(10);

    spindexer.optimizeBusUtilization();

    tower.getPosition().setUpdateFrequency(50);
    tower.getVelocity().setUpdateFrequency(50);
    tower.getSupplyCurrent().setUpdateFrequency(50);
    tower.getDeviceTemp().setUpdateFrequency(10);

    tower.optimizeBusUtilization();
  }

  public void setSpindexerOutput(double output) {
    spindexer.setControl(m_dutyReq.withOutput(output));
    // spindexer.setControl(m_vtcfReq.withOutput(output));
  }

  public void setTowerOutput(double output) {
    tower.setControl(m_dutyReq.withOutput(output));
    // tower.setControl(m_vtcfReq.withOutput(output));
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

  public Command reverseSpindexer() {
    return runOnce(() -> setSpindexerOutput(SPINDEXER_REVERSE_OUTPUT));
  }

  public Command reverseTower() {
    return runOnce(() -> setTowerOutput(TOWER_REVERSE_OUTPUT));
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
