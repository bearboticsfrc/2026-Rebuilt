package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RPM;
import static frc.robot.util.PhoenixUtil.applyConfig;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.CAN;
import frc.robot.Mechanism;
import frc.robot.Robot;
import frc.robot.test.SelfTestable;

public class Rollers extends Mechanism implements SelfTestable {

  private final CANBus canivore = new CANBus(CAN.NAME);

  private final TalonFX motor = new TalonFX(CAN.ROLLERS, canivore);

  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0.0).withEnableFOC(true);

  public final AngularVelocity ROLLER_SPEED = RPM.of(5000);

  public final AngularVelocity ROLLER_SPEED_SLOW = RPM.of(1000);
  private final double gearRatio = 1.11;

  private final StatusSignal<Current> motorSupplyCurrent = motor.getSupplyCurrent(false);
  private final StatusSignal<Current> motorStatorCurrent = motor.getStatorCurrent(false);
  private final StatusSignal<AngularVelocity> motorVelocity = motor.getVelocity(false);
  private final StatusSignal<Temperature> motorTemperature = motor.getDeviceTemp(false);
  private final StatusSignal<Double> motorClosedLoopError = motor.getClosedLoopError(false);

  private static final AngularVelocity VELOCITY_TOLERANCE = RPM.of(200);

  @Logged private boolean selfTestPassed = false;

  private DCMotorSim motorSimModel;

  public Rollers() {
    super("Rollers");
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.CurrentLimits.StatorCurrentLimit = 120;
    config.CurrentLimits.SupplyCurrentLimit = 60;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.Slot0.kV =
        0.133; // 12V / 90.1 RPS (6000 RPM / 1.11 gear ratio) — theoretical, tune down if it
    // overshoots
    config.Slot0.kS = 0.25; // static friction, tune on real robot
    config.Slot0.kP = 0.15; // V per RPS of error — rollers don't need tight regulation
    config.Slot0.kA = 0.0; // leave at 0, not needed for simple rollers

    // For rollers, kV does most of the work — if kV is tuned well, kP just handles disturbances
    // Start by tuning kS and kV until steady-state is accurate, then add a small kP if there's
    // residual error under load.

    config.Feedback.SensorToMechanismRatio = gearRatio;

    applyConfig(() -> motor.getConfigurator().apply(config), getName());

    optimizeCAN();

    if (Robot.isSimulation()) {
      motorSimModel =
          simulationInitKrakenX60(
              motor, gearRatio, 0.025, ChassisReference.CounterClockwise_Positive);
    }

    System.out.println(getName() + " Subsystem Initialized");
  }

  private void optimizeCAN() {
    motorSupplyCurrent.setUpdateFrequency(50);
    motorStatorCurrent.setUpdateFrequency(50);
    motorVelocity.setUpdateFrequency(100);
    motorTemperature.setUpdateFrequency(4);
    motorClosedLoopError.setUpdateFrequency(50);

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

    super.logFaults(motor);
  }

  private void setOutput(AngularVelocity velocity) {
    motor.setControl(velocityVoltage.withVelocity(velocity));
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

  private boolean isNearTarget(AngularVelocity target) {
    return motorVelocity.getValue().isNear(target, VELOCITY_TOLERANCE);
  }

  private Command selfTestAt(AngularVelocity target, String ntKey) {
    return runOnce(() -> setOutput(target))
        .andThen(Commands.waitUntil(() -> isNearTarget(target)).withTimeout(2.0))
        .andThen(
            runOnce(
                () -> {
                  selfTestPassed = isNearTarget(target);
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
    return selfTestAt(ROLLER_SPEED_SLOW, "Robot/Tests/rollers/slow")
        .withName(getName() + ".SelfTestSlow");
  }

  @Override
  public Command selfTestFast() {
    return selfTestAt(ROLLER_SPEED, "Robot/Tests/rollers/fast")
        .withName(getName() + ".SelfTestFast");
  }

  @Logged(name = "setpoint")
  public double getSetpoint() {
    return velocityVoltage.Velocity;
  }

  @Logged(name = "velocityRPM")
  public double getVelocityInRPM() {
    return motorVelocity.getValue().in(RPM);
  }

  @Logged(name = "closedLoopError")
  public double getClosedLoopError() {
    return motorClosedLoopError.getValue();
  }

  @Logged(name = "velocity")
  public AngularVelocity getVelocity() {
    return motorVelocity.getValue();
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
  public Temperature getTemperature() {
    return motorTemperature.getValue();
  }

  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic(motor, gearRatio, motorSimModel);
  }
}
