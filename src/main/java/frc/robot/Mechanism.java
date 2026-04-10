package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Mechanism extends SubsystemBase {

  private int periodicCount = 0; // used for periodic fault logging

  private boolean lastHasFault = false;

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

  public Mechanism(String name) {
    super(name);

    String lowerCaseName = name.toLowerCase();

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

  public void logFaults(TalonFX motor) {
    if (periodicCount++ % 12 != 0) { // every 240ms at 50Hz loop
      return;
    }
    // Read each signal once
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

    // Log sticky
    // DogLog.log(UNDERVOLTAGE_STICKY_KEY, stickyUndervoltage);
    // DogLog.log(BOOT_DURING_ENABLE_STICKY_KEY, stickyBootDuring);
    // DogLog.log(DEVICE_TEMP_STICKY_KEY, stickyDeviceTemp);
    // DogLog.log(HARDWARE_STICKY_KEY, stickyHardware);
    // DogLog.log(STATOR_CURR_LIMIT_STICKY_KEY, stickyStatorCurrLimit);
    // DogLog.log(BRIDGE_BROWNOUT_STICKY_KEY, stickyBridgeBrownout);

    // // Log live
    // DogLog.log(UNDERVOLTAGE_LIVE_KEY, liveUndervoltage);
    // DogLog.log(BOOT_DURING_ENABLE_LIVE_KEY, liveBootDuring);
    // DogLog.log(DEVICE_TEMP_LIVE_KEY, liveDeviceTemp);
    // DogLog.log(HARDWARE_LIVE_KEY, liveHardware);
    // DogLog.log(STATOR_CURR_LIMIT_LIVE_KEY, liveStatorCurrLimit);
    // DogLog.log(BRIDGE_BROWNOUT_LIVE_KEY, liveBridgeBrownout);

    // Summary
    boolean hasFault =
        liveUndervoltage || liveHardware || liveDeviceTemp || liveBootDuring || liveBridgeBrownout;
    //  DogLog.log(FAULT_SUMMARY_KEY, hasFault);

    // Report once on transition to faulted
    if (hasFault && !lastHasFault) {
      DriverStation.reportError(getName() + " has a fault! Check logs.", false);
    }
    lastHasFault = hasFault;
  }

  //
  // Simulation
  //
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
