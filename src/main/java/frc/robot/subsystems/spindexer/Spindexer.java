package frc.robot.subsystems.spindexer;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.PhoenixUtil.applyConfig;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
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
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CAN;
import frc.robot.Robot;
import frc.robot.test.SelfTestable;
import java.util.Set;

public class Spindexer extends SubsystemBase implements SelfTestable {

  private static final double kGearRatio = 7.2;

  private final CANBus canivore = new CANBus(CAN.NAME);
  private final TalonFX motor = new TalonFX(CAN.SPINDEXER, canivore);

  private final VelocityVoltage velocityReq =
      new VelocityVoltage(0.0).withEnableFOC(true).withSlot(0);
  private final MotionMagicVoltage positionReq =
      new MotionMagicVoltage(0.0).withEnableFOC(true).withSlot(1);

  private final AngularVelocity NORMAL_SPEED = RPM.of(600);
  private final AngularVelocity SLOW_SPEED = RPM.of(60);
  private final AngularVelocity REVERSE_SPEED = RPM.of(-200);

  private final StatusSignal<Current> supplyCurrent = motor.getSupplyCurrent(false);
  private final StatusSignal<Current> statorCurrent = motor.getStatorCurrent(false);
  private final StatusSignal<AngularVelocity> velocity = motor.getVelocity(false);
  private final StatusSignal<Angle> position = motor.getPosition(false);
  private final StatusSignal<Temperature> motorTemperature = motor.getDeviceTemp(false);
  private final StatusSignal<Double> motorClosedLoopError = motor.getClosedLoopError(false);

  private DCMotorSim simModel;

  public Spindexer() {
    super("Spindexer");

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.CurrentLimits.StatorCurrentLimit = 120;
    config.CurrentLimits.SupplyCurrentLimit = 60;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Velocity Control Gains
    config.Slot0.kS = 0.1;
    config.Slot0.kV = 0.75; // ~12V / (7530 RPM / 60 / 7.2) = 0.69
    config.Slot0.kA = 0;
    config.Slot0.kP = 0.15;

    // Position Control Gains (using Motion Magic)
    config.Slot1.kS = 0.1;
    config.Slot1.kV = 0.7; // ~12V / (7530 RPM / 60 / 7.2) = 0.69
    config.Slot1.kA = 0;
    config.Slot1.kP = 1;
    config.MotionMagic.MotionMagicCruiseVelocity = 0.5; // RPS, mechanism
    config.MotionMagic.MotionMagicAcceleration = 4; // RPS/s, mechanism
    config.Feedback.SensorToMechanismRatio = kGearRatio;

    applyConfig(() -> motor.getConfigurator().apply(config), getName());

    if (Robot.isSimulation()) {
      simulationInit();
    }

    optimizeCAN();
    System.out.println(getName() + " Subsystem Initialized");
  }

  private void optimizeCAN() {
    motor.getPosition().setUpdateFrequency(50);
    motor.getVelocity().setUpdateFrequency(50);
    motor.getSupplyCurrent().setUpdateFrequency(50);
    motor.getDeviceTemp().setUpdateFrequency(10);
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

  public Command oscillate() {
    return Commands.defer(
            () -> {
              Angle startPosition = position.getValue();
              Angle forwardTarget = startPosition.plus(Rotations.of(.25));
              Angle reverseTarget = startPosition;

              return run(() ->
                      motor.setControl(positionReq.withPosition(forwardTarget).withEnableFOC(true)))
                  .until(() -> position.getValue().isNear(forwardTarget, Rotations.of(.05)))
                  .andThen(
                      run(() ->
                              motor.setControl(
                                  positionReq.withPosition(reverseTarget).withEnableFOC(true)))
                          .until(
                              () -> position.getValue().isNear(reverseTarget, Rotations.of(.05))))
                  .repeatedly();
            },
            Set.of(this))
        .withName(getName() + ".oscillate");
  }

  @Logged private boolean selfTestPassed = false;
  private static final AngularVelocity SELF_TEST_VELOCITY_THRESHOLD_RPM = RPM.of(50);

  private boolean isNearTarget(AngularVelocity target) {
    return velocity.getValue().isNear(target, SELF_TEST_VELOCITY_THRESHOLD_RPM);
  }

  private Command selfTestAt(AngularVelocity target, String ntKey) {
    return runOnce(() -> motor.setControl(velocityReq.withVelocity(target)))
        .withName("Spindexer.SelfTestAt" + (int) target.in(RPM) + "RPM")
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
                  SmartDashboard.putBoolean(ntKey + "/passed", selfTestPassed);
                  SmartDashboard.putString(ntKey + "/message", result);
                }))
        .finallyDo(() -> motor.stopMotor());
  }

  @Override
  public Command selfTestSlow() {
    return selfTestAt(SLOW_SPEED, "Robot/Tests/spindexer/slow")
        .withName(getName() + ".SelfTestSlow");
  }

  @Override
  public Command selfTestFast() {
    return selfTestAt(NORMAL_SPEED, "Robot/Tests/spindexer/fast")
        .withName(getName() + ".SelfTestFast");
  }

  @Logged
  public AngularVelocity getVelocity() {
    return velocity.getValue();
  }

  @Logged(name = "velocityRPM")
  public double getVelocityInRPM() {
    return velocity.getValue().in(RPM);
  }

  @Logged(name = "setpointRPM")
  public double getSetpointInRPM() {
    return velocityReq.Velocity * 60.0;
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
  public Temperature getTemperature() {
    return motorTemperature.getValue();
  }

  @Logged(name = "closedLoopError")
  public double getClosedLoopError() {
    return motorClosedLoopError.getValue();
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(
        velocity, position, supplyCurrent, statorCurrent, motorTemperature, motorClosedLoopError);
  }

  // Simulation
  public void simulationInit() {
    var sim = motor.getSimState();
    sim.Orientation = ChassisReference.Clockwise_Positive;
    sim.setMotorType(TalonFXSimState.MotorType.KrakenX44);

    simModel =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44Foc(1), 0.01, kGearRatio),
            DCMotor.getKrakenX44Foc(1));
  }

  @Override
  public void simulationPeriodic() {
    var sim = motor.getSimState();
    sim.setSupplyVoltage(RobotController.getBatteryVoltage());
    var motorVoltage = sim.getMotorVoltageMeasure();
    simModel.setInputVoltage(motorVoltage.in(Volts));
    simModel.update(0.020);
    sim.setRawRotorPosition(simModel.getAngularPosition().times(kGearRatio));
    sim.setRotorVelocity(simModel.getAngularVelocity().times(kGearRatio));
  }
}
