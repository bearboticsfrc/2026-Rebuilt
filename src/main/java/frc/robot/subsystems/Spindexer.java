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
  private final TalonFX kicker = new TalonFX(23, canivore);

  private final DutyCycleOut m_dutyReq = new DutyCycleOut(0.0);
  private final VelocityTorqueCurrentFOC m_vtcfReq = new VelocityTorqueCurrentFOC(0.0);
  private final double KICKER_OUTPUT = 1;
  private final double KICKER_REVERSE_OUTPUT = -0.2;

  private final double SPINDEXER_OUTPUT = 0.7;
  private final double SPINDEXER_REVERSE_OUTPUT = -0.2;

  public Spindexer() {
    super("Spindexer");

    TalonFXConfiguration spindexerConfig = new TalonFXConfiguration();
    TalonFXConfiguration kickerConfig = new TalonFXConfiguration();

    spindexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    spindexerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    CurrentLimitsConfigs spindexerCurrentLimitsConfigs = new CurrentLimitsConfigs();

    spindexerCurrentLimitsConfigs.StatorCurrentLimit = 240;
    spindexerCurrentLimitsConfigs.SupplyCurrentLimit = 140;
    spindexerCurrentLimitsConfigs.StatorCurrentLimitEnable = true;
    spindexerCurrentLimitsConfigs.SupplyCurrentLimitEnable = true;
    spindexerConfig.withCurrentLimits(spindexerCurrentLimitsConfigs);

    kickerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    kickerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    CurrentLimitsConfigs kickerCurrentLimitsConfigs = new CurrentLimitsConfigs();
    kickerCurrentLimitsConfigs.StatorCurrentLimit = 240;
    kickerCurrentLimitsConfigs.SupplyCurrentLimit = 140;
    kickerCurrentLimitsConfigs.StatorCurrentLimitEnable = true;
    kickerCurrentLimitsConfigs.SupplyCurrentLimitEnable = true;
    kickerConfig.withCurrentLimits(kickerCurrentLimitsConfigs);

    StatusCode status = spindexer.getConfigurator().apply(spindexerConfig);

    for (int i = 0; i < 2; ++i) {
      if (status.isOK()) break;
      status = spindexer.getConfigurator().apply(spindexerConfig);
    }
    if (!status.isOK()) {
      System.out.println("ERROR Configuring Spindexer spindexer motor: " + status);
    }

    status = kicker.getConfigurator().apply(kickerConfig);
    for (int i = 0; i < 2; ++i) {
      if (status.isOK()) break;
      status = kicker.getConfigurator().apply(kickerConfig);
    }
    if (!status.isOK()) {
      System.out.println("ERROR Configuring Spindexer kicker motor: " + status);
    }

    optimizeCAN();
  }

  private void optimizeCAN() {
    spindexer.getPosition().setUpdateFrequency(50);
    spindexer.getVelocity().setUpdateFrequency(50);
    spindexer.getSupplyCurrent().setUpdateFrequency(50);
    spindexer.getDeviceTemp().setUpdateFrequency(10);

    spindexer.optimizeBusUtilization();

    kicker.getPosition().setUpdateFrequency(50);
    kicker.getVelocity().setUpdateFrequency(50);
    kicker.getSupplyCurrent().setUpdateFrequency(50);
    kicker.getDeviceTemp().setUpdateFrequency(10);

    kicker.optimizeBusUtilization();
  }

  public void setSpindexerOutput(double output) {
    spindexer.setControl(m_dutyReq.withOutput(output));
    // spindexer.setControl(m_vtcfReq.withOutput(output));
  }

  public void setKickerOutput(double output) {
<<<<<<< HEAD
   kicker.setControl(m_dutyReq.withOutput(output));
    // kicker.setControl(m_vtcfReq.withOutput(output));
=======
    kicker.setControl(m_dutyReq.withOutput(output));
    // tower.setControl(m_vtcfReq.withOutput(output));
>>>>>>> b45dab9bfedeca2f743805977d7e9d8039df53cb
  }

  public void stopMotors() {
    kicker.stopMotor();
    spindexer.stopMotor();
  }

  public Command runSpindexer() {
    return runOnce(() -> setSpindexerOutput(SPINDEXER_OUTPUT));
  }

  public Command runKicker() {
    return runOnce(() -> setKickerOutput(KICKER_OUTPUT));
  }

  public Command reverseSpindexer() {
    return runOnce(() -> setSpindexerOutput(SPINDEXER_REVERSE_OUTPUT));
  }

  public Command reverseKicker() {
    return runOnce(() -> setKickerOutput(KICKER_REVERSE_OUTPUT));
  }

  public Command run() {
    return runSpindexer().andThen(runKicker());
  }

  public Command stop() {
    return runOnce(() -> stopMotors());
  }

  @Logged
  public double getSpindexerVelocity() {
    //return spindexer velocity in RPM
    return spindexer.getVelocity().getValue().in(RPM);
  }

  @Logged
  public double getKickerVelocity() {
    //return kicker velocity in RPM
    return kicker.getVelocity().getValue().in(RPM);
  }
}
