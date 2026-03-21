package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rollers extends SubsystemBase {

  private final CANBus rio = new CANBus("rio");

  private final TalonFX motor = new TalonFX(25, rio);

  private final DutyCycleOut dutyReq = new DutyCycleOut(0.0);

  public final double ROLLER_SPEED = 0.65;

  public final double ROLLER_SPEED_SLOW = 0.3;

  private final StatusSignal<Current> motorSupplyCurrent = motor.getSupplyCurrent(false);
  private final StatusSignal<Current> motorStatorCurrent = motor.getStatorCurrent(false);
  private final StatusSignal<AngularVelocity> motorVelocity = motor.getVelocity(false);

  public Rollers() {
    super("Rollers");
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    CurrentLimitsConfigs motorCurrentLimitsConfigs = new CurrentLimitsConfigs();
    motorCurrentLimitsConfigs.StatorCurrentLimit = 120;
    motorCurrentLimitsConfigs.SupplyCurrentLimit = 60;
    motorCurrentLimitsConfigs.StatorCurrentLimitEnable = true;
    motorCurrentLimitsConfigs.SupplyCurrentLimitEnable = true;
    config.withCurrentLimits(motorCurrentLimitsConfigs);

    tryUntilOk(5, () -> motor.getConfigurator().apply(config), getName());

    System.out.println(getName() + " Subsystem Initialized");
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(motorSupplyCurrent, motorStatorCurrent, motorVelocity);
  }

  private void setOutput(double output) {
    motor.setControl(dutyReq.withOutput(output).withEnableFOC(true));
  }

  public Command run() {
    return runOnce(() -> setOutput(ROLLER_SPEED)).withName(getName() + ".Run");
  }

  public Command runSlow() {
    return runOnce(() -> setOutput(ROLLER_SPEED_SLOW)).withName(getName() + ".RunSlow");
  }

  public Command stop() {
    return runOnce(() -> motor.stopMotor()).withName(getName() + ".Stop");
  }

  @Logged
  public double getRollerVelocityInRPM() {
    return motorVelocity.getValue().in(RPM);
  }

  @Logged
  public double getRollerSupplyCurrent() {
    return motorSupplyCurrent.getValue().in(Amps);
  }

  @Logged
  public double getRollerStatorCurrent() {
    return motorStatorCurrent.getValue().in(Amps);
  }
}
