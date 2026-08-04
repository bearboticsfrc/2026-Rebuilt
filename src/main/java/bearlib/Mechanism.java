package bearlib;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.rebuilt.PhoenixUtil.applyConfig;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Base class for single-motor mechanism. For CTRE hardware, handles telemetry, logging, and sim.
 */
public class Mechanism extends SubsystemBase {

  private int periodicCount = 0; // used for periodic fault logging

  private boolean lastHasFault = false;

  protected TalonFX motor;

  protected TalonFXConfiguration motorConfig;

  protected final StatusSignal<Current> motorSupplyCurrent;
  protected final StatusSignal<Current> motorStatorCurrent;
  protected final StatusSignal<AngularVelocity> motorVelocity;
  protected final StatusSignal<Temperature> motorTemperature;
  protected final StatusSignal<Double> motorClosedLoopError;
  protected final StatusSignal<Voltage> motorVoltage;
  protected final StatusSignal<Angle> motorPosition;
  protected final StatusSignal<Double> setpoint;
  protected final StatusSignal<Double> motorProfileVelocity;

  /**
   * Constructor.
   *
   * @param name name of the mechanism.
   * @param ID The CAN ID.
   * @param canivore The CANBus.
   */
  public Mechanism(String name, int ID, CANBus canivore) {
    super(name);

    motor = new TalonFX(ID, canivore);

    motorConfig = new TalonFXConfiguration();

    motorSupplyCurrent = motor.getSupplyCurrent(false);
    motorStatorCurrent = motor.getStatorCurrent(false);
    motorVelocity = motor.getVelocity(false);
    motorTemperature = motor.getDeviceTemp(false);
    motorClosedLoopError = motor.getClosedLoopError(false);
    motorPosition = motor.getPosition(false);
    motorProfileVelocity = motor.getClosedLoopReferenceSlope(false);
    motorVoltage = motor.getMotorVoltage(false);
    setpoint = motor.getClosedLoopReference(false);
  }

  /** Refresh status signals, log faults. */
  @Override
  public void periodic() {
    /* refresh all status signals */
    BaseStatusSignal.refreshAll(
        motorPosition,
        motorVelocity,
        motorStatorCurrent,
        motorSupplyCurrent,
        motorVoltage,
        motorTemperature,
        setpoint);
    logFaults(motor);
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

  /**
   * Logs sticky and hardware faults,
   *
   * @param motor The TalonFX motor controller instance to check for faults.
   */
  public void logFaults(TalonFX motor) {
    if (periodicCount++ % 12 != 0) { // every 240ms at 50Hz loop
      return;
    }

    boolean liveUndervoltage = motor.getFault_Undervoltage().getValue();
    boolean liveBootDuring = motor.getFault_BootDuringEnable().getValue();
    boolean liveDeviceTemp = motor.getFault_DeviceTemp().getValue();
    boolean liveHardware = motor.getFault_Hardware().getValue();
    boolean liveBridgeBrownout = motor.getFault_BridgeBrownout().getValue();

    boolean hasFault =
        liveUndervoltage || liveHardware || liveDeviceTemp || liveBootDuring || liveBridgeBrownout;

    // Report once on transition to faulted
    if (hasFault && !lastHasFault) {
      DriverStation.reportError(getName() + " has a fault! Check logs.", false);
    }
    lastHasFault = hasFault;
  }

  /* CONFIG */

  /**
   * Sets and enables the stator current limit for the mechanism.
   *
   * @param value The stator current limit value.
   */
  public void statorCurrentLimit(double value) {
    motorConfig.CurrentLimits.StatorCurrentLimit = Amps.of(value).in(Amps);
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
  }

  /**
   * Sets and enables the supply current for the mechanism.
   *
   * @param value The supply current limit value.
   */
  public void supplyCurrentLimit(double value) {
    motorConfig.CurrentLimits.SupplyCurrentLimit = Amps.of(value).in(Amps);
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
  }

  /**
   * Sets the proportional gain of motor controller.
   *
   * @param p The propotional gain.
   */
  public void kP(double p) {
    motorConfig.Slot0.kP = p;
  }

  /**
   * Sets the integral gain of motor controller.
   *
   * @param i The integral gain.
   */
  public void kI(double i) {
    motorConfig.Slot0.kI = i;
  }

  /**
   * Sets the derivative gain of motor controller.
   *
   * @param d The derivative gain.
   */
  public void kD(double d) {
    motorConfig.Slot0.kD = d;
  }

  /**
   * Sets the static feedforward gain of motor controller.
   *
   * @param s The static feedforward gain.
   */
  public void kS(double s) {
    motorConfig.Slot0.kS = s;
  }

  /**
   * Sets the gravity feedback/forward gain of motor controller.
   *
   * @param g The gravity feedback/forward gain.
   */
  public void kG(double g) {
    motorConfig.Slot0.kG = g;
  }

  /**
   * Sets the acceleration feedforward gain of motor controller.
   *
   * @param a The acceleration feedforward gain.
   */
  public void kA(double a) {
    motorConfig.Slot0.kA = a;
  }

  /**
   * Sets the velocity feedforward gain of motor controller.
   *
   * @param a The velocity feedforward gain.
   */
  public void kV(double v) {
    motorConfig.Slot0.kV = v;
  }

  /**
   * Sets the neutral mode output for mechanism.
   *
   * @param neutralModeValue The neutral mode.
   */
  public void neutralMode(NeutralModeValue neutralModeValue) {
    motorConfig.MotorOutput.NeutralMode = neutralModeValue;
  }

  /**
   * Sets the inverted output for mechanism.
   *
   * @param invertedValue The inverted value.
   */
  public void inverted(InvertedValue invertedValue) {
    motorConfig.MotorOutput.Inverted = invertedValue;
  }

  /**
   * Sets the forward soft limit.
   *
   * @param softLimit The value of the soft limit.
   */
  public void forwardSoftLimit(double softLimit) {
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = softLimit;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
  }

  /**
   * Sets the reverse soft limit.
   *
   * @param softLimit The value of the soft limit.
   */
  public void reverseSoftLimit(double softLimit) {
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = softLimit;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
  }

  /**
   * Sets the motion magic acceleration.
   *
   * @param acceleration MotionMagic acceleration value.
   */
  public void motionMagicAcceleration(double acceleration) {
    motorConfig.MotionMagic.MotionMagicAcceleration = acceleration;
  }

  /**
   * Sets the motion magic velocity.
   *
   * @param velocity MotionMagic velocity value.
   */
  public void motionMagicCruiseVelocity(double velocity) {
    motorConfig.MotionMagic.MotionMagicCruiseVelocity = velocity;
  }

  /**
   * Sets the gravity type.
   *
   * @param gravityTypeValue The gravity value for the gravity type.
   */
  public void gravityType(GravityTypeValue gravityTypeValue) {
    motorConfig.Slot0.GravityType = gravityTypeValue;
  }

  /**
   * Sets the sensor to mechanism ratio.
   *
   * @param sensorToMechanismRatio The sensor to mechanism ratio.
   */
  public void sensorToMechanismRatio(double sensorToMechanismRatio) {
    motorConfig.Feedback.SensorToMechanismRatio = sensorToMechanismRatio;
  }

  /**
   * Sets the motion magic jerk.
   *
   * @param motionMagicJerk The motion magic jerk value.
   */
  public void motionMagicJerk(double motionMagicJerk) {
    motorConfig.MotionMagic.MotionMagicJerk = motionMagicJerk;
  }

  /** Applies config to motor. */
  public void addConfig() {
    applyConfig(() -> motor.getConfigurator().apply(motorConfig), getName());
  }

  /* LOGGED VALUES */

  /**
   * Logs closed loop error for mechanism.
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
   * Logs motor supply current for mechanism
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
   * @return double, motorTemperature
   */
  @Logged(name = "temperature")
  public double getTemperature() {
    return motorTemperature.getValue().in(Celsius);
  }

  /**
   * Logs motor voltage for mechanism.
   *
   * @return Voltage, motorVoltage
   */
  @Logged(name = "voltage")
  public Voltage getMotorVoltageMeasure() {
    return motorVoltage.getValue();
  }

  /**
   * Logs motor position for mechanism.
   *
   * @return double, motorPosition
   */
  @Logged(name = "position")
  public double getPosition() {
    return motorPosition.getValueAsDouble();
  }

  /**
   * Logs motor position as an angle for mechanism.
   *
   * @return Angle, motorPosition
   */
  @Logged(name = "angle")
  public Angle getAngle() {
    return motorPosition.getValue();
  }

  /**
   * Logs setpoint in degrees for mechanism.
   *
   * @return double, setpoint
   */
  @Logged(name = "setpointDegrees")
  public double getSetpointDegrees() {
    return setpoint.getValueAsDouble() * 360.0;
  }

  /**
   * Logs profile velocity in rps for mechanism.
   *
   * @return double, motorProfileVelocity
   */
  @Logged(name = "velocityRPS")
  public double getProfileVelocityRPS() {
    return motorProfileVelocity.getValue();
  }

  protected void optimizeCAN() {
    motorPosition.setUpdateFrequency(250);
    motorVelocity.setUpdateFrequency(250);
    motorSupplyCurrent.setUpdateFrequency(50);
    motorStatorCurrent.setUpdateFrequency(50);
    motorClosedLoopError.setUpdateFrequency(50);
    motorTemperature.setUpdateFrequency(4);
    motorProfileVelocity.setUpdateFrequency(50);

    motor.optimizeBusUtilization();
  }

  /* SIMULATION */

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
}
