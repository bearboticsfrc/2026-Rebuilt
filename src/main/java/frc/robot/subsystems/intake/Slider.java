package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.PhoenixUtil.applyConfig;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CAN;
import frc.robot.Robot;
import frc.robot.test.SelfTestable;
import java.util.function.Supplier;

public class Slider extends SubsystemBase implements SelfTestable {

  /** Position setpoints for the Slider. */
  public enum Setpoint {
    Retracted(Inches.of(0.02)),
    Middle(Inches.of(6)),
    Extended(Inches.of(11.2)); // max travel

    /** The position target as a mechanism angle (rotations of the pinion). */
    public final Angle target;

    /** The position target as a linear distance. */
    public final Distance targetDist;

    private Setpoint(Distance target) {
      this.targetDist = target;
      // distance = rotations * kPinionCircumference  →  rotations = distance / kPinionCircumference
      this.target = Rotations.of(target.in(Inches) / kPinionCircumference.in(Inches));
    }
  }

  private static final Distance kPinionCircumference = Inches.of(1.0 * Math.PI);

  private static final double gearRatio = 1.8; // 1.58; // 6.04; // motor-to-pinion gear reduction

  private final CANBus canivore = new CANBus(CAN.NAME);

  private final TalonFX motor = new TalonFX(CAN.SLIDER, canivore);

  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
  private final DutyCycleOut calibrateRequest = new DutyCycleOut(0).withIgnoreSoftwareLimits(true);

  /* device status signals */
  private final StatusSignal<Current> motorSupplyCurrent = motor.getSupplyCurrent(false);
  private final StatusSignal<Current> motorStatorCurrent = motor.getStatorCurrent(false);
  private final StatusSignal<Angle> motorPosition = motor.getPosition(false);
  private final StatusSignal<AngularVelocity> motorVelocity = motor.getVelocity(false);
  private final StatusSignal<Temperature> motorTemperature = motor.getDeviceTemp(false);
  private final StatusSignal<Double> motorClosedLoopError = motor.getClosedLoopError(false);

  private final StatusSignal<Double> sliderProfileVelocity =
      motor.getClosedLoopReferenceSlope(false);

  private DCMotorSim motorSimModel;

  public Slider() {
    super("Slider");

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.CurrentLimits.StatorCurrentLimit = Amps.of(150).in(Amps);
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.Slot0.kS = 0.7; //  1.0; // 1.5;
    config.Slot0.kV = 0.17;
    config.Slot0.kP = 3.0;
    config.Slot0.kD = 0.05;
    // config.Slot0.kI = 1.0;
    config.Slot0.kG = -0.3; // TODO: tune kG for tilt angle once mechanism angle is known
    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    config.Feedback.SensorToMechanismRatio = gearRatio;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Setpoint.Extended.target.in(Rotations);
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.00;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    config.MotionMagic.MotionMagicCruiseVelocity = RotationsPerSecond.of(20).in(RotationsPerSecond);
    config.MotionMagic.MotionMagicAcceleration =
        RotationsPerSecondPerSecond.of(240).in(RotationsPerSecondPerSecond);
    config.MotionMagic.MotionMagicJerk = 800;

    applyConfig(() -> motor.getConfigurator().apply(config), getName());

    motor.setPosition(Rotations.of(0)); // assume retracted at startup

    optimizeCAN();

    if (Robot.isSimulation()) {
      simulationInit();
    }

    System.out.println(getName() + " Subsystem Initialized");
  }

  private void optimizeCAN() {
    motorPosition.setUpdateFrequency(250);
    motorVelocity.setUpdateFrequency(250);
    motorSupplyCurrent.setUpdateFrequency(50);
    motorStatorCurrent.setUpdateFrequency(50);
    motorClosedLoopError.setUpdateFrequency(50);
    motorTemperature.setUpdateFrequency(4);

    motor.optimizeBusUtilization();
  }

  public Command extend() {
    return goToSetpoint(() -> Setpoint.Extended).withName(getName() + ".Extend");
  }

  public Command retract() {
    return goToSetpoint(() -> Setpoint.Retracted).withName(getName() + ".Retract");
  }

  public Command mid() {
    return goToSetpoint(() -> Setpoint.Middle).withName(getName() + ".Mid");
  }

  public Command stop() {
    return runOnce(() -> motor.stopMotor()).withName(getName() + ".Stop");
  }

  public Command lowOscillate() {
    return mid()
        .withTimeout(0.5)
        .andThen(retract().withTimeout(0.5))
        .andThen(mid())
        .withTimeout(0.5)
        .andThen(extend())
        .withTimeout(.5)
        .andThen(retract().withTimeout(0.5))
        .repeatedly()
        .withName(getName() + ".lowOscillate");
  }

  public Command highOscillate() {
    return mid()
        .withTimeout(0.5)
        .andThen(extend().withTimeout(0.5))
        .repeatedly()
        .withName(getName() + ".highOscillate");
  }

  /**
   * Drives the slider to the provided position setpoint.
   *
   * @param setpoint Function returning the setpoint to apply
   * @return Command to run
   */
  private Command goToSetpoint(Supplier<Setpoint> setpoint) {
    return run(
        () -> {
          motionMagicRequest.withPosition(setpoint.get().target);
          motor.setControl(motionMagicRequest);
        });
  }

  /**
   * Drives the slider to an arbitrary distance target.
   *
   * @param distance Function returning the target distance
   * @return Command to run
   */
  private Command goToDistance(Supplier<Distance> distance) {
    return run(
        () -> {
          double rotations = distance.get().in(Inches) / kPinionCircumference.in(Inches);
          motionMagicRequest.withPosition(Rotations.of(rotations));
          motor.setControl(motionMagicRequest);
        });
  }

  /**
   * Holds the slider at the current position using PID.
   *
   * @return Command to run
   */
  public Command holdPosition() {
    return runOnce(() -> motionMagicRequest.withPosition(motorPosition.getValue()))
        .andThen(run(() -> motor.setControl(motionMagicRequest)))
        .withName(getName() + ".HoldPosition");
  }

  private static final double kCalibrateOutput = -.12;
  private static final double kCalibrateStallAmps = 50.0;

  /**
   * Recalibrates the slider zero point. This slowly drives the slider up until we see a drop in
   * velocity and a spike in stator current, indicating that we've hit a hard stop.
   *
   * @return Command to run
   */
  public Command calibrateZero() {
    return run(() -> {
          calibrateRequest.withOutput(kCalibrateOutput);

          motor.setControl(calibrateRequest);
        })
        .until(() -> motorStatorCurrent.getValue().in(Amps) > kCalibrateStallAmps)
        .withTimeout(3.0)
        .andThen(
            runOnce(
                () -> {
                  motor.stopMotor();
                  motor.setPosition(Rotations.of(0));
                  isZeroed = true;
                }))
        .withName(getName() + ".CalibrateZero");
  }

  @Logged private boolean isZeroed = false;

  @Logged private boolean selfTestPassed = false;
  private static final double SELF_TEST_TOLERANCE_INCHES = 0.5;

  // TODO: safeguard the position of the slider, should start at 0
  // TODO: retract slider at end of test
  private Command selfTestAt(Setpoint target, String ntKey) {
    return Commands.runOnce(
            () -> {
              var nt = NetworkTableInstance.getDefault();
              nt.getEntry(ntKey + "/message").setString("Running...");
              nt.getEntry(ntKey + "/passed").unpublish();
              ;
            })
        .andThen(goToSetpoint(() -> target))
        .withName(getName() + ".TestSetpoint" + target.name())
        .withTimeout(4.0)
        .andThen(
            runOnce(
                () -> {
                  double errorInches = Math.abs(getPositionInches() - target.targetDist.in(Inches));
                  selfTestPassed = errorInches < SELF_TEST_TOLERANCE_INCHES;
                  String result =
                      (selfTestPassed ? "PASS" : "FAIL")
                          + ": "
                          + String.format("%.2f", getPositionInches())
                          + "\" (target "
                          + String.format("%.2f", target.targetDist.in(Inches))
                          + ")";
                  var nt = NetworkTableInstance.getDefault();
                  nt.getEntry(ntKey + "/passed").setBoolean(selfTestPassed);
                  nt.getEntry(ntKey + "/message").setString(result);
                }))
        .andThen(retract().withTimeout(2.0))
        .finallyDo(() -> motor.stopMotor());
  }

  @Override
  public Command selfTestSlow() {
    return selfTestAt(Setpoint.Middle, "Robot/Tests/slider/slow")
        .withName(getName() + ".SelfTestSlow");
  }

  @Override
  public Command selfTestFast() {
    return selfTestAt(Setpoint.Extended, "Robot/Tests/slider/fast")
        .withName(getName() + ".SelfTestFast");
  }

  /**
   * @return Current position in mechanism rotations (pinion rotations).
   */
  @Logged(name = "position")
  public double getPosition() {
    return motorPosition.getValue().in(Rotations);
  }

  @Logged(name = "setpoint")
  public double getSetpoint() {
    return motionMagicRequest.Position;
  }

  /**
   * @return Current position converted to linear distance.
   */
  @Logged
  public Distance getPositionDistance() {
    return Inches.of(getPositionInches());
  }

  /**
   * @return Current position in inches.
   */
  @Logged(name = "positionInches")
  public double getPositionInches() {
    return motorPosition.getValue().in(Rotations) * kPinionCircumference.in(Inches);
  }

  @Logged
  public double getProfileVelocityRPS() {
    return sliderProfileVelocity.getValue();
  }

  /**
   * @return The MotionMagic setpoint in inches.
   */
  @Logged(name = "setpointInches")
  public double getSetpointInches() {
    return motionMagicRequest.getPositionMeasure().in(Rotations) * kPinionCircumference.in(Inches);
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
  public double getTemperature() {
    return motorTemperature.getValue().in(Celsius);
  }

  @Logged(name = "closedLoopError")
  public double getClosedLoopError() {
    return motorClosedLoopError.getValue();
  }

  /**
   * @return true if the slider is at or past the extended setpoint.
   */
  @Logged
  public boolean isExtended() {
    return getPositionInches()
        >= Setpoint.Extended.targetDist.in(Inches) - SELF_TEST_TOLERANCE_INCHES;
  }

  @Logged
  public boolean isRetracted() {
    return getPositionInches()
        <= Setpoint.Retracted.targetDist.in(Inches) + SELF_TEST_TOLERANCE_INCHES;
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(
        motorSupplyCurrent,
        motorStatorCurrent,
        motorPosition,
        motorVelocity,
        motorTemperature,
        motorClosedLoopError);
  }

  //
  // Simulation
  //
  public void simulationInit() {
    var talonFXSim = motor.getSimState();

    talonFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
    talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX44);

    motorSimModel =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44Foc(1), 0.0012, gearRatio),
            DCMotor.getKrakenX44Foc(1));

    var simConfig = new TalonFXConfiguration();
    motor.getConfigurator().refresh(simConfig);
    simConfig.Slot0.kS = 0.0;
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
    talonFXSim.setRawRotorPosition(motorSimModel.getAngularPosition().times(gearRatio));
    talonFXSim.setRotorVelocity(motorSimModel.getAngularVelocity().times(gearRatio));
  }
}
