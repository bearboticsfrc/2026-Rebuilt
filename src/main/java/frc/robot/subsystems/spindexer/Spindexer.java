package frc.robot.subsystems.spindexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.CAN.*;
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
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CAN;
import frc.robot.Robot;

public class Spindexer extends SubsystemBase {

  private final CANBus canivore = new CANBus(CAN.NAME);

  private final TalonFX spindexer = new TalonFX(CAN.SPINDEXER, canivore);
  private final TalonFX kicker = new TalonFX(CAN.KICKER, canivore);

  private final DutyCycleOut kickerDutyReq = new DutyCycleOut(0.0);
  private final DutyCycleOut spindexerDutyReq = new DutyCycleOut(0.0);
  private final double KICKER_OUTPUT = 1;
  private final double KICKER_REVERSE_OUTPUT = -0.2;

  private final double SPINDEXER_OUTPUT = 0.7;
  private final double SPINDEXER_REVERSE_OUTPUT = -0.2;

  private final StatusSignal<Current> spindexerSupplyCurrent = spindexer.getSupplyCurrent(false);
  private final StatusSignal<Current> spindexerStatorCurrent = spindexer.getStatorCurrent(false);
  private final StatusSignal<AngularVelocity> spindexerVelocity = spindexer.getVelocity(false);

  private final StatusSignal<Current> kickerSupplyCurrent = kicker.getSupplyCurrent(false);
  private final StatusSignal<Current> kickerStatorCurrent = kicker.getStatorCurrent(false);
  private final StatusSignal<AngularVelocity> kickerVelocity = kicker.getVelocity(false);

  private static final double kickerGearRatio = 2.5;
  private static final double spindexerGearRatio = 7.2;

  private DCMotorSim kickerSimModel;
  private DCMotorSim spindexerSimModel;

  public Spindexer() {
    super("Spindexer");

    TalonFXConfiguration spindexerConfig = new TalonFXConfiguration();
    spindexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    spindexerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    CurrentLimitsConfigs spindexerCurrentLimitsConfigs = new CurrentLimitsConfigs();
    spindexerCurrentLimitsConfigs.StatorCurrentLimit = 120;
    spindexerCurrentLimitsConfigs.SupplyCurrentLimit = 60;
    spindexerCurrentLimitsConfigs.StatorCurrentLimitEnable = true;
    spindexerCurrentLimitsConfigs.SupplyCurrentLimitEnable = true;
    spindexerConfig.withCurrentLimits(spindexerCurrentLimitsConfigs);

    TalonFXConfiguration kickerConfig = new TalonFXConfiguration();
    kickerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    kickerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    CurrentLimitsConfigs kickerCurrentLimitsConfigs = new CurrentLimitsConfigs();
    kickerCurrentLimitsConfigs.StatorCurrentLimit = 120;
    kickerCurrentLimitsConfigs.SupplyCurrentLimit = 60;
    kickerCurrentLimitsConfigs.StatorCurrentLimitEnable = true;
    kickerCurrentLimitsConfigs.SupplyCurrentLimitEnable = true;
    kickerConfig.withCurrentLimits(kickerCurrentLimitsConfigs);

    tryUntilOk(5, () -> spindexer.getConfigurator().apply(spindexerConfig), getName());
    tryUntilOk(5, () -> kicker.getConfigurator().apply(kickerConfig), getName());

    if (Robot.isSimulation()) {
      simulationInit();
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

  private void setSpindexerOutput(double output) {
    spindexer.setControl(spindexerDutyReq.withOutput(output).withEnableFOC(true));
  }

  private void setKickerOutput(double output) {
    kicker.setControl(kickerDutyReq.withOutput(output).withEnableFOC(true));
  }

  private void stopMotors() {
    kicker.stopMotor();
    spindexer.stopMotor();
  }

  public Command runSpindexer() {
    return runOnce(() -> setSpindexerOutput(SPINDEXER_OUTPUT)).withName("RunSpindexer");
  }

  public Command runKicker() {
    return runOnce(() -> setKickerOutput(KICKER_OUTPUT)).withName("RunKicker");
  }

  public Command reverseSpindexer() {
    return runOnce(() -> setSpindexerOutput(SPINDEXER_REVERSE_OUTPUT)).withName("ReverseSpindexer");
  }

  public Command reverseKicker() {
    return runOnce(() -> setKickerOutput(KICKER_REVERSE_OUTPUT)).withName("ReverseKicker");
  }

  public Command run() {
    return runSpindexer().andThen(runKicker()).withName("SpindexerAndKicker");
  }

  public Command stop() {
    return runOnce(() -> stopMotors()).withName("StopSpindexerAndKicker");
  }

  @Logged
  public double getSpindexerVelocity() {
    return spindexerVelocity.getValue().in(RPM);
  }

  @Logged
  public double getKickerVelocity() {
    return kickerVelocity.getValue().in(RPM);
  }

  @Logged
  public double getSpindexerSupplyCurrent() {
    return spindexerSupplyCurrent.getValue().in(Amps);
  }

  @Logged
  public double getSpindexerStatorCurrent() {
    return spindexerStatorCurrent.getValue().in(Amps);
  }

  @Logged
  public double getKickerStatorCurrent() {
    return kickerStatorCurrent.getValue().in(Amps);
  }

  @Logged
  public double getKickerSupplyCurrent() {
    return kickerSupplyCurrent.getValue().in(Amps);
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(
        spindexerVelocity,
        kickerVelocity,
        spindexerSupplyCurrent,
        spindexerStatorCurrent,
        kickerStatorCurrent,
        kickerSupplyCurrent);
  }

  // Simulation
  public void simulationInit() {
    spindexerSimulationInit();
    kickerSimulationInit();
  }

  public void spindexerSimulationInit() {
    var spindexerTalonFXSim = spindexer.getSimState();

    // Match your InvertedValue.Clockwise_Positive config
    spindexerTalonFXSim.Orientation = ChassisReference.Clockwise_Positive;
    spindexerTalonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX44);

    spindexerSimModel =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX44Foc(1), 0.01, spindexerGearRatio),
            DCMotor.getKrakenX44Foc(1));
  }

  public void kickerSimulationInit() {
    var kickerTalonFXSim = spindexer.getSimState();

    // Match your InvertedValue.Clockwise_Positive config
    kickerTalonFXSim.Orientation = ChassisReference.Clockwise_Positive;
    kickerTalonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);

    kickerSimModel =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.025, kickerGearRatio),
            DCMotor.getKrakenX60Foc(1));
  }

  @Override
  public void simulationPeriodic() {
    spindexerSimulationPeriodic();
    kickerSimulationPeriodic();
  }

  public void spindexerSimulationPeriodic() {
    var spindexerTalonFXSim = spindexer.getSimState();

    // set the supply voltage of the TalonFX
    spindexerTalonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    // get the motor voltage of the TalonFX
    var motorVoltage = spindexerTalonFXSim.getMotorVoltageMeasure();

    // use the motor voltage to calculate new position and velocity
    // using WPILib's DCMotorSim class for physics simulation
    spindexerSimModel.setInputVoltage(motorVoltage.in(Volts));
    spindexerSimModel.update(0.020); // assume 20 ms loop time

    // apply the new rotor position and velocity to the TalonFX;
    // note that this is rotor position/velocity (before gear ratio), but
    // DCMotorSim returns mechanism position/velocity (after gear ratio)
    spindexerTalonFXSim.setRawRotorPosition(
        spindexerSimModel.getAngularPosition().times(spindexerGearRatio));
    spindexerTalonFXSim.setRotorVelocity(
        spindexerSimModel.getAngularVelocity().times(spindexerGearRatio));
  }

  public void kickerSimulationPeriodic() {
    var kickerTalonFXSim = spindexer.getSimState();

    // set the supply voltage of the TalonFX
    kickerTalonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    // get the motor voltage of the TalonFX
    var motorVoltage = kickerTalonFXSim.getMotorVoltageMeasure();

    // use the motor voltage to calculate new position and velocity
    // using WPILib's DCMotorSim class for physics simulation
    kickerSimModel.setInputVoltage(motorVoltage.in(Volts));
    kickerSimModel.update(0.020); // assume 20 ms loop time

    // apply the new rotor position and velocity to the TalonFX;
    // note that this is rotor position/velocity (before gear ratio), but
    // DCMotorSim returns mechanism position/velocity (after gear ratio)
    kickerTalonFXSim.setRawRotorPosition(
        spindexerSimModel.getAngularPosition().times(kickerGearRatio));
    kickerTalonFXSim.setRotorVelocity(
        spindexerSimModel.getAngularVelocity().times(kickerGearRatio));
  }
}
