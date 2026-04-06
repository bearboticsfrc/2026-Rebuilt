package frc.robot.subsystems.spindexer;

import static edu.wpi.first.units.Units.Celsius;
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

public class Kicker extends Mechanism implements SelfTestable {

  private static final double kGearRatio = 2.5;

  private final CANBus canivore = new CANBus(CAN.NAME);
  private final TalonFX motor = new TalonFX(CAN.KICKER, canivore);

  private final VelocityVoltage velocityReq = new VelocityVoltage(0.0).withEnableFOC(true);

  private final AngularVelocity NORMAL_SPEED = RPM.of(2200);
  private final AngularVelocity SLOW_SPEED = RPM.of(200);
  private final AngularVelocity REVERSE_SPEED = RPM.of(-200);

  private final StatusSignal<Current> supplyCurrent = motor.getSupplyCurrent(false);
  private final StatusSignal<Current> statorCurrent = motor.getStatorCurrent(false);
  private final StatusSignal<AngularVelocity> velocity = motor.getVelocity(false);
  private final StatusSignal<Temperature> motorTemperature = motor.getDeviceTemp(false);
  private final StatusSignal<Double> motorClosedLoopError = motor.getClosedLoopError(false);

  private DCMotorSim simModel;

  public Kicker() {
    super("Kicker");

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.CurrentLimits.StatorCurrentLimit = 120;
    config.CurrentLimits.SupplyCurrentLimit = 60;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.Slot0.kS = 0.25;
    config.Slot0.kV = 0.3; // ~12V / (40 RPS) = 0.3
    config.Slot0.kA = 0.0;
    config.Slot0.kP = 0.1;

    /*
      Tuning order:
      1. Set only kS and kV, run at your target RPM — check if steady-state velocity is close
      2. If there's consistent error under load (fuel backed up), increase kV slightly or add a small kP
      3. kS is the one most likely to need adjustment — too low and it won't start moving from rest, too high and it jerks on startup

      Since the kicker is downstream of the spindexer and only sees the fuel briefly, tight velocity control matters less here than on the flywheel. Getting kV approximately right is sufficient
      for consistent behavior.
    */
    config.Feedback.SensorToMechanismRatio = kGearRatio;

    applyConfig(() -> motor.getConfigurator().apply(config), getName());

    if (Robot.isSimulation()) {
      simModel =
          simulationInitKrakenX60(motor, kGearRatio, 0.025, ChassisReference.Clockwise_Positive);
    }

    optimizeCAN();
    System.out.println(getName() + " Subsystem Initialized");
  }

  private void optimizeCAN() {
    supplyCurrent.setUpdateFrequency(50);
    statorCurrent.setUpdateFrequency(50);
    velocity.setUpdateFrequency(250);
    motorTemperature.setUpdateFrequency(10);
    motorClosedLoopError.setUpdateFrequency(50);

    motor.optimizeBusUtilization();
  }

  public Command run() {
    return runOnce(() -> motor.setControl(velocityReq.withVelocity(NORMAL_SPEED)))
        .withName(getName() + ".Run");
  }

  public Command reverse() {
    return runOnce(() -> motor.setControl(velocityReq.withVelocity(REVERSE_SPEED)))
        .withName(getName() + ".Reverse");
  }

  public Command stop() {
    return runOnce(() -> stopMotor()).withName(getName() + ".Stop");
  }

  public void stopMotor() {
    motor.stopMotor();
  }

  @Logged private boolean selfTestPassed = false;
  private static final AngularVelocity SELF_TEST_VELOCITY_THRESHOLD_RPM = RPM.of(200);

  private boolean isNearTarget(AngularVelocity target) {
    return velocity.getValue().isNear(target, SELF_TEST_VELOCITY_THRESHOLD_RPM);
  }

  private Command selfTestAt(AngularVelocity target, String ntKey) {
    return Commands.runOnce(
            () -> {
              var nt = NetworkTableInstance.getDefault();
              nt.getEntry(ntKey + "/message").setString("Running...");
              nt.getEntry(ntKey + "/passed").unpublish();
              ;
            })
        .andThen(runOnce(() -> motor.setControl(velocityReq.withVelocity(target))))
        .withName(getName() + ".SelfTestAt" + (int) target.in(RPM) + "RPM")
        .andThen(Commands.waitSeconds(1))
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
    return selfTestAt(SLOW_SPEED, "Robot/Tests/kicker/slow").withName(getName() + ".SelfTestSlow");
  }

  @Override
  public Command selfTestFast() {
    return selfTestAt(NORMAL_SPEED, "Robot/Tests/kicker/fast")
        .withName(getName() + ".SelfTestFast");
  }

  @Logged(name = "velocityRPM")
  public double getVelocityInRPM() {
    return velocity.getValue().in(RPM);
  }

  @Logged(name = "setpointRPM")
  public double getSetpointInRPM() {
    return velocityReq.Velocity * 60.0;
  }

  @Logged(name = "velocity")
  public AngularVelocity getVelocity() {
    return velocity.getValue();
  }

  @Logged(name = "closedLoopError")
  public double getClosedLoopError() {
    return motorClosedLoopError.getValue();
  }

  @Logged(name = "supplyCurrent")
  public Current getSupplyCurrent() {
    return supplyCurrent.getValue();
  }

  @Logged(name = "statorCurrent")
  public Current getStatorCurrent() {
    return statorCurrent.getValue();
  }

  @Logged(name = "temperature")
  public double getTemperature() {
    return motorTemperature.getValue().in(Celsius);
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(
        velocity, supplyCurrent, statorCurrent, motorTemperature, motorClosedLoopError);
    super.logFaults(motor);
  }

  // Simulation
  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic(motor, kGearRatio, simModel);
  }
}
