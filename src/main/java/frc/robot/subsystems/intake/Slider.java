package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import bearlib.Mechanism;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.rebuilt.CAN;
import frc.robot.test.SelfTestable;
import java.util.function.Supplier;

public class Slider extends Mechanism implements SelfTestable {

  /** Position setpoints for the Slider. */
  public enum Setpoint {
    Retracted(Inches.of(0.2)),
    Middle(Inches.of(6)),
    Extended(Inches.of(10.8)); // max travel

    /** The position target as a mechanism angle (rotations of the pinion). */
    public Angle target;

    /** The position target as a linear distance. */
    public Distance targetDist;

    private Setpoint(Distance target) {
      this.targetDist = target;
      this.target = Rotations.of(target.in(Inches) / kPinionCircumference.in(Inches));
    }
  }

  private static final Distance kPinionCircumference = Inches.of(1.0 * Math.PI);

  private static final double gearRatio = 1.8;

  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
  private final DutyCycleOut calibrateRequest = new DutyCycleOut(0).withIgnoreSoftwareLimits(true);

  private DCMotorSim motorSimModel;

  private final double STALL_VELOCITY_RPS = 0.05;
  private final double STALL_CURRENT_AMPS = 50.0;
  private final double STALL_TIME_SECONDS = 0.15;

  private final Timer stallTimer = new Timer();
  private boolean isStalled = false;
  private boolean hasStalled = false;

  @Logged private boolean isZeroed = false;
  @Logged private boolean selfTestPassed = false;
  private static final double SELF_TEST_TOLERANCE_INCHES = 0.5;

  private static final double kCalibrateOutput = -.12;
  private static final double kCalibrateStallAmps = 50.0;

  private boolean isCalibrating = false;

  public Slider() {
    super("Slider", CAN.SLIDER, new CANBus(CAN.NAME));

    neutralMode(NeutralModeValue.Brake);
    inverted(InvertedValue.CounterClockwise_Positive);
    statorCurrentLimit(Amps.of(70).in(Amps));
    supplyCurrentLimit(Amps.of(50).in(Amps));
    kS(0.7);
    kV(0.17);
    kP(3.0);
    kD(0.05);
    kG(-0.3);
    gravityType(GravityTypeValue.Elevator_Static);
    sensorToMechanismRatio(gearRatio);
    forwardSoftLimit(Setpoint.Extended.target.in(Rotations));
    reverseSoftLimit(0.00);
    motionMagicCruiseVelocity(RotationsPerSecond.of(20).in(RotationsPerSecond));
    motionMagicAcceleration(RotationsPerSecondPerSecond.of(240).in(RotationsPerSecondPerSecond));
    motionMagicJerk(800);

    addConfig();

    motor.setPosition(Rotations.of(0)); // assume retracted at startup

    optimizeCAN();

    if (Robot.isSimulation()) {
      motorSimModel = simulationInitKrakenX44(motor, gearRatio, SELF_TEST_TOLERANCE_INCHES, null);
    }

    System.out.println(getName() + " Subsystem Initialized");
  }

  /** Runs the slider to the extended setpoint. */
  public Command extend() {
    return goToSetpoint(() -> Setpoint.Extended).withName(getName() + ".Extend");
  }

  /** Runs the slider to the retracted setpoint. */
  public Command retract() {
    return goToSetpoint(() -> Setpoint.Retracted).withName(getName() + ".Retract");
  }

  /** Runs the slider to the middle setpoint. */
  public Command mid() {
    return goToSetpoint(() -> Setpoint.Middle).withName(getName() + ".Mid");
  }

  /** Stops the slider. */
  public Command stop() {
    return runOnce(() -> motor.stopMotor()).withName(getName() + ".Stop");
  }

  /** Oscilates the slider. */
  public Command lowOscillate() {
    return mid()
        .withTimeout(0.75)
        .andThen(retract().withTimeout(1.25))
        .andThen(mid())
        .withTimeout(0.75)
        .andThen(retract().withTimeout(1.25))
        .repeatedly()
        .withName(getName() + ".lowOscillate");
  }

  /**
   * Drives the slider to the provided position setpoint.
   *
   * @param setpoint Function returning the setpoint to apply
   * @return Command to run
   */
  private Command goToSetpoint(Supplier<Setpoint> setpoint) {
    return Commands.runEnd(
        () -> {
          if (isStalled || hasStalled) {
            motionMagicRequest.withPosition(motorPosition.getValue());
            hasStalled = true;
          } else {
            motionMagicRequest.withPosition(setpoint.get().target);
          }
          motor.setControl(motionMagicRequest);
        },
        () -> {
          hasStalled = false;
        },
        this);
  }

  /** Subsytem periodic. */
  public void periodic() {
    super.periodic();
    checkIfStalled();
  }

  /** Signals whether or not the slider is stalled. */
  public boolean checkIfStalled() {

    if (Math.abs(getVelocity().in(RotationsPerSecond)) < STALL_VELOCITY_RPS
        && getStatorCurrent().in(Amps) > STALL_CURRENT_AMPS) {
      if (stallTimer.get() == 0) {
        stallTimer.start();
      }

      if (stallTimer.hasElapsed(STALL_TIME_SECONDS)) {
        isStalled = true;
      }
    } else {
      stallTimer.stop();
      stallTimer.reset();
      isStalled = false;
    }

    return isStalled;
  }

  /**
   * Recalibrates the slider zero point. This slowly drives the slider up until we see a drop in
   * velocity and a spike in stator current, indicating that we've hit a hard stop.
   *
   * @return Command to run
   */
  public Command calibrateZero() {
    return runOnce(() -> isZeroed = false)
        .andThen(
            run(
                () -> {
                  calibrateRequest.withOutput(kCalibrateOutput);
                  isCalibrating = true;
                  motor.setControl(calibrateRequest);
                }))
        .until(() -> motorStatorCurrent.getValue().in(Amps) > kCalibrateStallAmps)
        .withTimeout(3.0)
        .andThen(
            runOnce(
                () -> {
                  motor.stopMotor();
                  motor.setPosition(Rotations.of(0));
                  isZeroed = true;
                }))
        .finallyDo(() -> isCalibrating = false)
        .withName(getName() + ".CalibrateZero");
  }

  /**
   * Self test at target speed.
   *
   * @param target The target speed.
   * @param ntKey The NT key.
   */
  private Command selfTestAt(Setpoint target, String ntKey) {
    return Commands.runOnce(
            () -> {
              var nt = NetworkTableInstance.getDefault();
              nt.getEntry(ntKey + "/message").setString("Running...");
              nt.getEntry(ntKey + "/passed").unpublish();
            })
        .andThen(calibrateZero())
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

  /** Self test for slider at slow speed. */
  @Override
  public Command selfTestSlow() {
    return selfTestAt(Setpoint.Middle, "Robot/Tests/slider/slow")
        .withName(getName() + ".SelfTestSlow");
  }

  /** Self test for slider at fast speed. */
  @Override
  public Command selfTestFast() {
    return selfTestAt(Setpoint.Extended, "Robot/Tests/slider/fast")
        .withName(getName() + ".SelfTestFast");
  }

  /** The setpoint of the slider. */
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

  /**
   * @return The MotionMagic setpoint in inches.
   */
  @Logged(name = "setpointInches")
  public double getSetpointInches() {
    return motionMagicRequest.getPositionMeasure().in(Rotations) * kPinionCircumference.in(Inches);
  }

  /**
   * @return true if the slider is at or past the extended setpoint.
   */
  @Logged
  public boolean isExtended() {
    return getPositionInches()
        >= Setpoint.Extended.targetDist.in(Inches) - SELF_TEST_TOLERANCE_INCHES + 0.4;
  }

  /** Signals if the slider is retracted. */
  @Logged
  public boolean isRetracted() {
    return getPositionInches()
        <= Setpoint.Retracted.targetDist.in(Inches) + SELF_TEST_TOLERANCE_INCHES;
  }

  /*
   * Signals if the slider is stopped.
   */
  @Logged
  public boolean isStopped() {
    return motor.getMotorVoltage().getValueAsDouble() <= 0.5;
  }

  /** Signals if the slider is zeroed. */
  @Logged
  public boolean isZeroed() {
    return isZeroed;
  }

  /** Signals if the slider is calibrating */
  @Logged
  public boolean isCalibrating() {
    return isCalibrating;
  }

  /** Signals if slider is stalled. */
  @Logged
  public boolean isStalled() {
    return isStalled;
  }

  /** Simulation periodic. */
  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic(motor, gearRatio, motorSimModel);
  }
}
