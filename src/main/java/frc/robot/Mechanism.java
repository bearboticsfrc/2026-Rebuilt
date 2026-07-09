package frc.robot;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A base subsystem class representing a generic motorized mechanism utilizing a TalonFX motor
 * controller. Handles telemetry keys, fault logging, and physics-based simulation setup.
 */
public class Mechanism extends SubsystemBase {

  private int periodicCount = 0; // used for periodic fault logging

  private boolean lastHasFault = false;

  protected TalonFX motor;

  protected StatusSignal<Current> motorSupplyCurrent;
  protected StatusSignal<Current> motorStatorCurrent;
  protected StatusSignal<AngularVelocity> motorVelocity;
  protected StatusSignal<Temperature> motorTemperature;
  protected StatusSignal<Double> motorClosedLoopError;

  private final String UNDERVOLTAGE_STICKY_KEY;
  private final String BOOT_DURING_ENABLE_STICKY_KEY;
  private final String DEVICE_TEMP_STICKY_KEY;
  private final String HARDWARE_STICKY_KEY;
  private final String STATOR_CURR_LIMIT_STICKY_KEY;
  private final String BRIDGE_BROWNOUT_STICKY_KEY;

  private final String UNDERVOLTAGE_LIVE_KEY;
  private final String BOOT_DURING_ENABLE_LIVE_KEY;
  private final String DEVICE_TEMP_LIVE_KEY;
  private final String HARDWARE_LIVE_KEY;
  private final String STATOR_CURR_LIMIT_LIVE_KEY;
  private final String BRIDGE_BROWNOUT_LIVE_KEY;

  private final String FAULT_SUMMARY_KEY;

  /**
   * Constructs a new Mechanism subsystem.
   *
   * @param name The user-friendly name of the mechanism (used for telemetry logging paths).
   * @param ID The CAN ID of the TalonFX motor controller.
   * @param canivore The CANBus network where the motor controller resides.
   */
  public Mechanism(String name, int ID, CANBus canivore) {
    super(name);

    String lowerCaseName = name.toLowerCase();

    motor = new TalonFX(ID, canivore);

    motorSupplyCurrent = motor.getSupplyCurrent(false);
    motorStatorCurrent = motor.getStatorCurrent(false);
    motorVelocity = motor.getVelocity(false);
    motorTemperature = motor.getDeviceTemp(false);
    motorClosedLoopError = motor.getClosedLoopError(false);

    UNDERVOLTAGE_STICKY_KEY = lowerCaseName + "/faults/sticky/undervoltage";
    BOOT_DURING_ENABLE_STICKY_KEY = lowerCaseName + "/faults/sticky/bootDuringEnable";
    DEVICE_TEMP_STICKY_KEY = lowerCaseName + "/faults/sticky/deviceTemp";
    HARDWARE_STICKY_KEY = lowerCaseName + "/faults/sticky/hardware";
    STATOR_CURR_LIMIT_STICKY_KEY = lowerCaseName + "/faults/sticky/statorCurrLimit";
    BRIDGE_BROWNOUT_STICKY_KEY = lowerCaseName + "/faults/sticky/bridgeBrownout";

    UNDERVOLTAGE_LIVE_KEY = lowerCaseName + "/faults/live/undervoltage";
    BOOT_DURING_ENABLE_LIVE_KEY = lowerCaseName + "/faults/live/bootDuringEnable";
    DEVICE_TEMP_LIVE_KEY = lowerCaseName + "/faults/live/deviceTemp";
    HARDWARE_LIVE_KEY = lowerCaseName + "/faults/live/hardware";
    STATOR_CURR_LIMIT_LIVE_KEY = lowerCaseName + "/faults/live/statorCurrLimit";
    BRIDGE_BROWNOUT_LIVE_KEY = lowerCaseName + "/faults/live/bridgeBrownout";
    FAULT_SUMMARY_KEY = lowerCaseName + "/hasFault";
  }

  /**
   * Periodically checks and logs live and sticky hardware faults. Throttled internally to run once
   * every 240ms (at a 50Hz loop rate). Reports an error to the DriverStation upon a new fault
   * transition.
   *
   * @param motor The TalonFX motor controller instance to check for faults.
   */
  public void logFaults(TalonFX motor) {
    if (periodicCount++ % 12 != 0) { // every 240ms at 50Hz loop
      return;
    }

    boolean stickyUndervoltage = motor.getStickyFault_Undervoltage().getValue();
    boolean stickyBootDuring = motor.getStickyFault_BootDuringEnable().getValue();
    boolean stickyDeviceTemp = motor.getStickyFault_DeviceTemp().getValue();
    boolean stickyHardware = motor.getStickyFault_Hardware().getValue();
    boolean stickyStatorCurrLimit = motor.getStickyFault_StatorCurrLimit().getValue();
    boolean stickyBridgeBrownout = motor.getStickyFault_BridgeBrownout().getValue();

    boolean liveUndervoltage = motor.getFault_Undervoltage().getValue();
    boolean liveBootDuring = motor.getFault_BootDuringEnable().getValue();
    boolean liveDeviceTemp = motor.getFault_DeviceTemp().getValue();
    boolean liveHardware = motor.getFault_Hardware().getValue();
    boolean liveStatorCurrLimit = motor.getFault_StatorCurrLimit().getValue();
    boolean liveBridgeBrownout = motor.getFault_BridgeBrownout().getValue();

    boolean hasFault =
        liveUndervoltage || liveHardware || liveDeviceTemp || liveBootDuring || liveBridgeBrownout;

    // Report once on transition to faulted
    if (hasFault && !lastHasFault) {
      DriverStation.reportError(getName() + " has a fault! Check logs.", false);
    }
    lastHasFault = hasFault;
  }

  /**
   * Initializes simulation model for a Kraken X60.
   *
   * @param motor The TalonFX motor controller being simulated.
   * @param gearRatio The gear ratio of the mechanism (motor rotations per mechanism rotation).
   * @param inertia The moment of inertia of the mechanism load (kg*m^2).
   * @param orientation The mechanical orientation of the motor relative to the chassis.
   * @return A {@link DCMotorSim} model representing the physical system.
   */
  public DCMotorSim simulationInitKrakenX60(
      TalonFX motor, double gearRatio, double inertia, ChassisReference orientation) {
    TalonFXSimState talonFXSim = motor.getSimState();

    talonFXSim.Orientation = orientation;
    talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);

    DCMotorSim motorSimModel =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), inertia, gearRatio),
            DCMotor.getKrakenX60Foc(1));

    var simConfig = new TalonFXConfiguration();
    motor.getConfigurator().refresh(simConfig);
    simConfig.Slot0.kS = 0.0;
    motor.getConfigurator().apply(simConfig);

    return motorSimModel;
  }

  /**
   * Logs closed loop error for mechanism
   *
   * @return double, closedLoopError
   */
  @Logged(name = "closedLoopError")
  public double getClosedLoopError() {
    return motorClosedLoopError.getValue();
  }

  /**
   * Logs motor velocity for mechansim
   *
   * @return AngularVelocity, motorVelocity
   */
  @Logged(name = "velocity")
  public AngularVelocity getVelocity() {
    return motorVelocity.getValue();
  }

  /**
   * Logs motor supplu current for mechanism
   *
   * @return Current, motorSupplyCurrent
   */
  @Logged(name = "supplyCurrent")
  public Current getSupplyCurrent() {
    return motorSupplyCurrent.getValue();
  }

  /**
   * Logs motor stator current for mechanism
   *
   * @return Current, motorStatorCurrent
   */
  @Logged(name = "statorCurrent")
  public Current getStatorCurrent() {
    return motorStatorCurrent.getValue();
  }

  /**
   * Logs motor temperature for mechanism
   *
   * @return double, mototTemperature
   */
  @Logged(name = "temperature")
  public double getTemperature() {
    return motorTemperature.getValue().in(Celsius);
  }

  /**
   * Initializes simulation for Kraken X44 motor.
   *
   * @param motor The TalonFX motor controller being simulated.
   * @param gearRatio The gear ratio of the mechanism (motor rotations per mechanism rotation).
   * @param inertia The moment of inertia of the mechanism load (kg*m^2).
   * @param orientation The mechanical orientation of the motor relative to the chassis.
   * @return A {@link DCMotorSim} model representing the physical system.
   */
  public DCMotorSim simulationInitKrakenX44(
      TalonFX motor, double gearRatio, double inertia, ChassisReference orientation) {
    TalonFXSimState talonFXSim = motor.getSimState();

    talonFXSim.Orientation = orientation;
    talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX44);

    DCMotorSim motorSimModel =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44Foc(1), inertia, gearRatio),
            DCMotor.getKrakenX44Foc(1));

    var simConfig = new TalonFXConfiguration();
    motor.getConfigurator().refresh(simConfig);
    simConfig.Slot0.kS = 0.0;
    motor.getConfigurator().apply(simConfig);

    return motorSimModel;
  }

  /**
   * Updates the simulation for the mechanism.
   *
   * @param motor The TalonFX motor controller being simulated.
   * @param gearRatio The gear ratio of the mechanism.
   * @param motorSimModel The physics simulation model associated with this mechanism.
   */
  public void simulationPeriodic(TalonFX motor, double gearRatio, DCMotorSim motorSimModel) {
    var talonFXSim = motor.getSimState();

    // set the supply voltage of the TalonFX
    talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    // get the motor voltage of the TalonFX
    var motorVoltage = talonFXSim.getMotorVoltageMeasure();

    // use the motor voltage to calculate new position and velocity
    // using WPILib's DCMotorSim class for physics simulation
    motorSimModel.setInputVoltage(motorVoltage.in(Volts));
    motorSimModel.update(0.020); // assume 20 ms loop time

    // apply the new rotor position and velocity to the TalonFX;
    // note that this is rotor position/velocity (before gear ratio), but
    // DCMotorSim returns mechanism position/velocity (after gear ratio)
    talonFXSim.setRawRotorPosition(motorSimModel.getAngularPosition().times(gearRatio));
    talonFXSim.setRotorVelocity(motorSimModel.getAngularVelocity().times(gearRatio));
  }
}
