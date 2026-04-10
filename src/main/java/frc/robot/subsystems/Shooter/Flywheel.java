// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.PhoenixUtil.applyConfig;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.CAN;
import frc.robot.Robot;
import frc.robot.test.SelfTestable;
import java.util.function.DoubleSupplier;

public class Flywheel extends SubsystemBase implements SelfTestable {
  /** Creates a new Flywheel. */
  // Create a new CANBus with name canivore
  private final CANBus canivore = new CANBus(CAN.NAME);

  private final TalonFX motor = new TalonFX(CAN.FLYWHEEL, canivore);

  // Velocity output control for the flywheel

  private final MotionMagicVelocityVoltage velocityOut = new MotionMagicVelocityVoltage(0);

  // Tolerance for the flywheel velocity
  private final double tolerance = 750; // RPM

  private final VoltageOut m_voltReq = new VoltageOut(0.0);

  private final StatusSignal<Current> motorSupplyCurrent = motor.getSupplyCurrent(false);
  private final StatusSignal<Current> motorStatorCurrent = motor.getStatorCurrent(false);
  private final StatusSignal<AngularVelocity> motorVelocity = motor.getVelocity(false);
  private final StatusSignal<Temperature> motorTemperature = motor.getDeviceTemp(false);
  private final StatusSignal<Double> motorClosedLoopError = motor.getClosedLoopError(false);

  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with Phoenix SignalLogger class
              (state) -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              (volts) -> motor.setControl(m_voltReq.withOutput(volts.in(Volts))), null, this));

  private DCMotorSim motorSimModel;
  private static final double SIM_GEAR_RATIO = 1.0;

  public Flywheel() {
    super("Flywheel");

    TalonFXConfiguration config = new TalonFXConfiguration();
    // Put's the motor in Coast mode to make it easier to move by hand
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.Slot0.kS = 0.31576; // Static gain
    config.Slot0.kV = 0.11941; // Velocity gain
    config.Slot0.kA = 0.015595;
    config.Slot0.kP = .185; // Proportional gain
    config.MotionMagic.MotionMagicCruiseVelocity = 9000; // Max velocity
    config.MotionMagic.MotionMagicAcceleration = 9000; // Max acceleration allowed
    config.TorqueCurrent.PeakForwardTorqueCurrent = 100;
    config.TorqueCurrent.PeakReverseTorqueCurrent = 0;
    config.MotorOutput.PeakForwardDutyCycle = 1;
    config.MotorOutput.PeakReverseDutyCycle = 0;
    config.CurrentLimits.StatorCurrentLimit = Amps.of(80).in(Amps);
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = Amps.of(60).in(Amps);
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Try to apply config multiple time. Break after successfully applying
    applyConfig(() -> motor.getConfigurator().apply(config), getName());

    if (Robot.isSimulation()) {
      simulationInit();
    }

    System.out.println(getName() + " Subsystem Initialized");

    optimizeCAN();
  }

  private void optimizeCAN() {
    motorVelocity.setUpdateFrequency(250);
    motorSupplyCurrent.setUpdateFrequency(50);
    motorStatorCurrent.setUpdateFrequency(50);
    motorClosedLoopError.setUpdateFrequency(50);
    motorTemperature.setUpdateFrequency(10);

    motor.optimizeBusUtilization();
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(
        motorSupplyCurrent,
        motorStatorCurrent,
        motorVelocity,
        motorTemperature,
        motorClosedLoopError);
  }

  public void setVelocity(AngularVelocity velocity) {
    // motor.setControl(new VelocityDutyCycle(velocity));
    motor.setControl(velocityOut.withVelocity(velocity));
  }

  /**
   * Command to run the flywheel at a given speed.
   *
   * @return The command to run the flywheel at the given speed.
   */
  public Command runAtSpeed(double rpm) {
    // Command to run the flywheel at a given speed
    return runOnce(() -> setVelocity(RPM.of(rpm))).withName(getName() + ".runAtSpeed(double)");
  }

  public Command warmUp() {
    return runAtSpeed(() -> 1600.0);
  }

  /**
   * Command to run the flywheel at a given speed.
   *
   * @return The command to run the flywheel at the given speed.
   */
  public Command runAtSpeed(DoubleSupplier rpm) {
    // Command to run the flywheel at a given speed
    return run(() -> setVelocity(RPM.of(rpm.getAsDouble())))
        .withName(getName() + ".runAtSpeed(supplier)");
  }

  /**
   * Command to stop the flywheel.
   *
   * @return The command to stop the flywheel.
   */
  public Command stopCommand() {
    return runOnce(() -> stop());
  }

  /**
   * Checks if the flywheel is at its target speed.
   *
   * @return true if at target speed, false otherwise
   */
  @Logged
  public boolean isAtTarget() {
    return getTargetVelocityInRPM() > 0
        && Math.abs(getVelocityInRPM() - getTargetVelocityInRPM())
            < tolerance; // Check if the current velocity is near the target velocity
  }

  @Logged(name = "closedLoopError")
  public double getClosedLoopError() {
    return motorClosedLoopError.getValue();
  }

  @Logged(name = "supplyCurrent")
  public Current getSupplyCurrent() {
    return motorSupplyCurrent.getValue();
  }

  @Logged(name = "statorCurrent")
  public Current getStatorCurrent() {
    return motorStatorCurrent.getValue();
  }

  @Logged(name = "temperature")
  public double getTemperature() {
    return motorTemperature.getValue().in(Celsius);
  }

  // Stop the flywheel motors
  public void stop() {
    motor.stopMotor();
  }

  @Logged private boolean selfTestPassed = false;

  private Command selfTestAt(AngularVelocity target, String ntKey) {
    return Commands.runOnce(
            () -> {
              var nt = NetworkTableInstance.getDefault();
              nt.getEntry(ntKey + "/message").setString("Running...");
              nt.getEntry(ntKey + "/passed").unpublish();
              ;
            })
        .andThen(runAtSpeed(() -> target.in(RPM)))
        .withTimeout(4)
        .andThen(Commands.waitUntil(this::isAtTarget).withTimeout(2.0))
        .andThen(
            runOnce(
                () -> {
                  selfTestPassed = isAtTarget();
                  String result =
                      (selfTestPassed ? "PASS" : "FAIL")
                          + ": "
                          + (int) getVelocityInRPM()
                          + " RPM (target "
                          + (int) target.in(RPM)
                          + " RPM)";
                  var nt = NetworkTableInstance.getDefault();
                  nt.getEntry(ntKey + "/passed").setBoolean(selfTestPassed);
                  nt.getEntry(ntKey + "/message").setString(result);
                }))
        .finallyDo(() -> motor.stopMotor());
  }

  @Override
  public Command selfTestSlow() {
    return selfTestAt(RPM.of(1000), "Robot/Tests/flywheel/slow")
        .withName(getName() + ".SelfTestSlow");
  }

  @Override
  public Command selfTestFast() {
    return selfTestAt(RPM.of(3150), "Robot/Tests/flywheel/fast")
        .withName(getName() + ".SelfTestFast");
  }

  /**
   * Returns a command that will execute a quasistatic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  @Logged(name = "velocity")
  public AngularVelocity getVelocity() {
    return motorVelocity.getValue();
  }

  @Logged(name = "velocityRPM")
  public double getVelocityInRPM() {
    return motorVelocity.getValue().in(RPM);
  }

  @Logged(name = "targetVelocityRPM")
  public double getTargetVelocityInRPM() {
    return velocityOut.getVelocityMeasure().in(RPM);
  }

  //
  // Simulation
  //
  public void simulationInit() {
    var talonFXSim = motor.getSimState();

    // Match your InvertedValue.Clockwise_Positive config
    talonFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
    talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);

    motorSimModel =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.004, SIM_GEAR_RATIO),
            DCMotor.getKrakenX60Foc(1));

    var simConfig = new TalonFXConfiguration();
    motor.getConfigurator().refresh(simConfig);
    simConfig.Slot0.kS = 0.2;
    motor.getConfigurator().apply(simConfig);
  }

  @Override
  public void simulationPeriodic() {
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
    talonFXSim.setRawRotorPosition(motorSimModel.getAngularPosition().times(SIM_GEAR_RATIO));
    talonFXSim.setRotorVelocity(motorSimModel.getAngularVelocity().times(SIM_GEAR_RATIO));
  }
}
