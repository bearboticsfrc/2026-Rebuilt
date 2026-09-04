package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import bearlib.Mechanism;
import bearlib.util.AllianceFlipUtil;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.rebuilt.CAN;
import frc.robot.subsystems.shooter.DynamicShootingCalculator;
import frc.robot.test.SelfTestable;
import java.util.function.Supplier;
import lombok.Getter;

public class Turret extends Mechanism implements NTSendable, SelfTestable {

  private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);
  private PositionVoltage positionVoltage = new PositionVoltage(0);
  private PositionVoltage positionHold = new PositionVoltage(0).withSlot(1);

  @Getter private double currentLimit = 44;
  @Getter private double torqueCurrentLimit = 400;

  @Getter private double gearRatio = 10.44;

  @Getter public Angle minRotations = Rotations.of(-.25);
  @Getter public Angle maxRotations = Rotations.of(.25);

  @Getter public boolean attached = true;

  @Logged private boolean selfTestPassed = false;

  private static final double SELF_TEST_VARIANCE_THRESHOLD = 0.01;

  private DCMotorSim motorSimModel;

  private Voltage kV;

  private static final double LARGE_JUMP_THRESHOLD = 0.2;

  public Turret() {
    super("Turret", CAN.TURRET, new CANBus(CAN.NAME));

    neutralMode(NeutralModeValue.Brake);
    inverted(InvertedValue.Clockwise_Positive);
    kP(70);
    kD(3);
    kA(0.05);
    kS(.6);
    kV(1.25);
    kP1(150);
    kD1(12);
    kA1(0);
    kS1(0);
    kV1(0);
    rotorToSensorRatio(1.0);
    sensorToMechanismRatio(gearRatio);
    supplyCurrentLimit(80);
    statorCurrentLimit(80);
    peakForwardTorqueCurrent(120);
    peakReverseTorqueCurrent(-120);
    forwardSoftLimit(.62);
    reverseSoftLimit(-.62);
    addConfig();
    motionMagicCruiseVelocity(3.0);
    motionMagicAcceleration(20);
    motionMagicJerk(0);
    motor.setPosition(0);

    addConfig();

    optimizeCAN();

    if (Robot.isSimulation()) {
      motorSimModel =
          simulationInitKrakenX44(motor, gearRatio, 0.025, ChassisReference.Clockwise_Positive);
    }

    System.out.println(getName() + " Subsystem Initialized");
  }

  /** Simulation periodic. */
  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic(motor, gearRatio, motorSimModel);
  }

  /**
   * Converts degree setpoint to valid one within soft limits.
   *
   * @param targetAngle The setpoint angle.
   */
  private Angle wrapDegreesToSoftLimits(Angle targetAngle) {
    Angle currentAngle = getAngle();

    int nMin = (int) Math.ceil(minRotations.minus(targetAngle).in(Degrees) / 360.0);
    int nMax = (int) Math.floor(maxRotations.minus(targetAngle).in(Degrees) / 360.0);

    if (nMin <= nMax) {
      int nClosest = (int) Math.round((currentAngle.minus(targetAngle).in(Degrees)) / 360.0);
      int n =
          Math.max(nMin, Math.min(nClosest, nMax)); // clamp the closest candidate to allowed range
      return Degrees.of(targetAngle.in(Degrees) + n * 360.0);
    } else {
      double toMin = Math.abs(currentAngle.minus(minRotations).in(Degrees));
      double toMax = Math.abs(currentAngle.minus(maxRotations).in(Degrees));
      return (toMin < toMax) ? minRotations : maxRotations;
    }
  }

  /*
   * Initialize NT sendable.
   */
  @Override
  public void initSendable(NTSendableBuilder builder) {}

  protected String getCurrentCommandName() {
    Command currentCommand = this.getCurrentCommand();
    if (currentCommand != null) {
      return currentCommand.getName();
    }

    return "none";
  }

  /** The motor position in degrees */
  @Logged
  public double getPositionDegrees() {
    return motorPosition.getValue().in(Degrees);
  }

  /** The setpoint measure in rotations. */
  @Logged(name = "setpointRotations")
  public double getSetpointRotations() {
    return setpoint.getValueAsDouble();
  }

  /** The motion magic setpoint measure in rotations. */
  @Logged(name = "motionMagicSetpointRotations")
  public double getMotionMagicSetpointRotations() {
    return motionMagicVoltage.Position;
  }

  /** The position voltage measure in rotations. */
  @Logged(name = "positionVoltageSetpointRotations")
  public double getPositionVoltageSetpointRotations() {
    return positionVoltage.Position;
  }

  /** The position hold setpoint measure in rotations. */
  @Logged(name = "positionHoldSetpointRotations")
  public double getPositionHoldSetpointRotations() {
    return positionHold.Position;
  }

  /** Stops the turret. */
  public Command stop() {
    return runOnce(() -> motor.stopMotor()).withName(this.getName() + ".Stop");
  }

  /**
   * Sets the turret to an angle using voltage control.
   *
   * @param angle The setpoint angle.
   */
  private void controlMotor(Angle angle) {
    motor.setControl(motionMagicVoltage.withPosition(wrapDegreesToSoftLimits(angle)));
  }

  /**
   * Sets the turret to an angle usinng voltge control with velocity feedforward.
   *
   * @param angle The setpoint angle.
   * @param velocity The velocity feedforward.
   */
  private void controlMotor(Angle angle, AngularVelocity velocity) {
    Angle target = wrapDegreesToSoftLimits(angle);
    double errorRotations = Math.abs(motorPosition.getValue().minus(target).in(Rotations));

    if (errorRotations > LARGE_JUMP_THRESHOLD) {
      motor.setControl(motionMagicVoltage.withPosition(target));
    } else {
      Voltage velocityFeedForward = kV.times(velocity.in(RotationsPerSecond));

      motor.setControl(positionVoltage.withPosition(target).withFeedForward(velocityFeedForward));
    }
  }

  /**
   * Sets the turret angle.
   *
   * @param angle The setpoint angle.
   */
  public Command setAngle(Angle angle) {
    return run(() -> controlMotor(angle)).withName(this.getName() + ".SetAngle");
  }

  /**
   * Sets the turret angle.
   *
   * @param angle The setpoint angle.
   */
  public Command setAngle(Supplier<Angle> angle) {
    return run(() -> controlMotor(angle.get())).withName(this.getName() + ".SetAngleSupplier");
  }

  /**
   * Sets the turret angle
   *
   * @param angle The turret angle.
   * @param velocity velocity feedforward.
   */
  public Command setAngle(Supplier<Angle> angle, Supplier<AngularVelocity> velocity) {
    return run(() -> controlMotor(angle.get(), velocity.get()))
        .withName(this.getName() + ".SetAngleSupplierWithVelocity");
  }

  /**
   * The angle from the turret to another location.
   *
   * @param position The other location coordinates.
   */
  public Angle getAngleTo(Translation2d position) {
    return (AllianceFlipUtil.apply(
            position.minus(RobotState.getInstance().getRobotPose().getTranslation()).getAngle())
        .getMeasure()
        .minus(
            AllianceFlipUtil.apply(RobotState.getInstance().getRobotPose().getRotation())
                .getMeasure()));
  }

  /** The targeted position of the turret. */
  @Logged
  public Translation2d getTargetedHubPosition() {
    Transform2d targetTransform =
        new Transform2d(
            new Translation2d(RobotState.getInstance().getLookaheadDistanceToHub(), 0.0),
            Rotation2d.kZero);

    Pose2d turretPose =
        DynamicShootingCalculator.getInstance()
            .getLookaheadPose()
            .transformBy(new Transform2d(0, 0, new Rotation2d(getAngle())));

    Translation2d targetPos = turretPose.transformBy(targetTransform).getTranslation();
    return targetPos;
  }

  /**
   * Signals whether the turret is at a certain setpoint.
   *
   * @param target The setpoint angle.
   */
  public boolean isNearTarget(Angle target) {
    return getAngle().isNear(target, SELF_TEST_VARIANCE_THRESHOLD);
  }

  /**
   * Self tests the turret.
   *
   * @param target The setpoint target.
   * @param ntKey The NT key.
   */
  private Command selfTestAt(Angle target, String ntKey) {
    return Commands.runOnce(
            () -> {
              var nt = NetworkTableInstance.getDefault();
              nt.getEntry(ntKey + "/message").setString("Running...");
              nt.getEntry(ntKey + "/passed").unpublish();
              ;
            })
        .andThen(setAngle(() -> target, () -> RadiansPerSecond.of(0.1)))
        .withTimeout(2.0)
        .andThen(
            runOnce(
                () -> {
                  selfTestPassed = isNearTarget(target);
                  String result =
                      (selfTestPassed ? "PASS" : "FAIL")
                          + ": "
                          + String.format("%.2f", getAngle().in(Degrees))
                          + " Deg (target "
                          + String.format("%.2f", target.in(Degrees))
                          + " Deg)";
                  var nt = NetworkTableInstance.getDefault();
                  nt.getEntry(ntKey + "/passed").setBoolean(selfTestPassed);
                  nt.getEntry(ntKey + "/message").setString(result);
                }))
        .andThen(setAngle(() -> Rotations.of(0)).withTimeout(1.0))
        .finallyDo(() -> motor.stopMotor());
  }

  /** Self tests slowly. */
  @Override
  public Command selfTestSlow() {
    return selfTestAt(Rotations.of(0.1), "Robot/Tests/turret/slow")
        .withName(getName() + ".SelfTestSlow");
  }

  /** Self tests at normal speed. */
  @Override
  public Command selfTestFast() {
    return selfTestAt(Rotations.of(.25), "Robot/Tests/turret/fast")
        .withName(getName() + ".SelfTestFast");
  }
}
