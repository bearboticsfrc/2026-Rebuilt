package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.PhoenixUtil.applyConfig;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.CAN;
import frc.robot.Copilot;
import frc.robot.Mechanism;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.field.AllianceFlipUtil;
import frc.robot.subsystems.DynamicShootingCalculator;
import frc.robot.test.SelfTestable;
import java.util.function.Supplier;
import limelight.Limelight;
import lombok.Getter;

public class Turret extends Mechanism implements NTSendable, SelfTestable {

  private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);
  private PositionVoltage positionVoltage = new PositionVoltage(0);
  private PositionVoltage positionHold = new PositionVoltage(0).withSlot(1);

  /* Turret config values */
  @Getter private double currentLimit = 44;
  @Getter private double torqueCurrentLimit = 400;

  @Getter private double gearRatio = 10.44;

  private final StatusSignal<Voltage> motorVoltage = motor.getMotorVoltage(false);
  private final StatusSignal<Angle> motorPosition = motor.getPosition(false);
  private final StatusSignal<Double> setpoint = motor.getClosedLoopReference(false);

  // set these small to start
  @Getter public Angle minRotations = Rotations.of(-.25);
  @Getter public Angle maxRotations = Rotations.of(.25);

  @Getter public boolean attached = true;

  @Logged private boolean selfTestPassed = false;

  // percent error tolerance for self test
  private static final double SELF_TEST_VARIANCE_THRESHOLD = 0.01;

  //  private final TurretVisionHelper limelight;

  private DCMotorSim motorSimModel;

  private Voltage kV;

  public Turret() {
    super("Turret", CAN.TURRET, new CANBus(CAN.NAME));

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.Slot0.kP =
        70; // 23.221; //   72 for 5 degrees, 180 for 2 degrees, 360 for 1 degrees.  Increase D
    // with
    // increase in P
    config.Slot0.kD = 3; // .8981; //   start with d = p / 100 and increase until oscillation stops

    config.Slot0.kA = 0.05; // .077265;
    config.Slot0.kS = .6; // .2; // start with .4; tune up if mechanism stalls at end of moves
    config.Slot0.kV =
        1.25; //  ( 0.124 x 4.34 = .54  V/mechanism-RPS ) // not used in positionvoltage
    kV = Volts.of(10.0); // consider zeroing this or zeroing the Slot0.kV and using only this

    config.Slot1.kP = 150;
    config.Slot1.kD = 12;
    config.Slot1.kA = 0;
    config.Slot1.kS = 0;
    config.Slot1.kV = 0;

    // var motionMagicConfigs = config.MotionMagic;
    config.MotionMagic.MotionMagicCruiseVelocity = 3.0; // 1.8; // Target cruise velocity of 80 rps
    config.MotionMagic.MotionMagicAcceleration =
        20; // 12.0; // 3.6; // Target acceleration of 160 rps/s (0.5 seconds)
    config.MotionMagic.MotionMagicJerk =
        0; // 40; // 25; // Target jerk of 1600 rps/s/s (0.1 seconds)

    config.Feedback.RotorToSensorRatio = 1.0;
    config.Feedback.SensorToMechanismRatio = gearRatio;

    config.CurrentLimits.SupplyCurrentLimit = 80;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 80;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.TorqueCurrent.PeakForwardTorqueCurrent = 120; // amps -- tune up from here
    config.TorqueCurrent.PeakReverseTorqueCurrent = -120;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = .62;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -.62;

    applyConfig(() -> motor.getConfigurator().apply(config), getName());

    motor.setPosition(0);

    optimizeCAN();

    if (Robot.isSimulation()) {
      motorSimModel =
          simulationInitKrakenX44(motor, gearRatio, 0.025, ChassisReference.Clockwise_Positive);
    }

    System.out.println(getName() + " Subsystem Initialized");
  }

  protected void optimizeCAN() {
    if (Robot.isSimulation()) {
      motorVelocity.setUpdateFrequency(1000);
      motorPosition.setUpdateFrequency(1000);
      setpoint.setUpdateFrequency(1000);

    } else {
      motorVelocity.setUpdateFrequency(250);
      motorPosition.setUpdateFrequency(250);
      setpoint.setUpdateFrequency(50);
    }
    motorSupplyCurrent.setUpdateFrequency(50);
    motorStatorCurrent.setUpdateFrequency(50);
    motorVoltage.setUpdateFrequency(50);
    motorTemperature.setUpdateFrequency(10);

    motor.optimizeBusUtilization();
  }

  private Angle wrapDegreesToSoftLimits(Angle targetAngle) {
    Angle currentAngle = getAngle();

    // Solve for integer n such that minRotations <= targetDegrees + 360*n <= maxRotations
    int nMin = (int) Math.ceil(minRotations.minus(targetAngle).in(Degrees) / 360.0);
    int nMax = (int) Math.floor(maxRotations.minus(targetAngle).in(Degrees) / 360.0);

    if (nMin <= nMax) {
      // At least one equivalent fits in soft limits.
      int nClosest = (int) Math.round((currentAngle.minus(targetAngle).in(Degrees)) / 360.0);
      int n =
          Math.max(nMin, Math.min(nClosest, nMax)); // clamp the closest candidate to allowed range
      return Degrees.of(targetAngle.in(Degrees) + n * 360.0);
    } else {
      // No equivalent fits in soft limits -> clamp to nearest soft limit endpoint.
      double toMin = Math.abs(currentAngle.minus(minRotations).in(Degrees));
      double toMax = Math.abs(currentAngle.minus(maxRotations).in(Degrees));
      return (toMin < toMax) ? minRotations : maxRotations;
    }
  }

  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic(motor, gearRatio, motorSimModel);
  }

  @Override
  public void initSendable(NTSendableBuilder builder) {}

  protected String getCurrentCommandName() {
    Command currentCommand = this.getCurrentCommand();
    if (currentCommand != null) {
      return currentCommand.getName();
    }

    return "none";
  }

  /* Logged values */

  /**
   * Updates the position of the motor
   *
   * @return motor position in degrees
   */
  @Logged
  public double getPositionDegrees() {
    return motorPosition.getValue().in(Degrees);
  }

  @Logged(name = "setpointRotations")
  public double getSetpointRotations() {
    return setpoint.getValueAsDouble();
  }

  @Logged(name = "motionMagicSetpointRotations")
  public double getMotionMagicSetpointRotations() {
    return motionMagicVoltage.Position;
  }

  @Logged(name = "positionVoltageSetpointRotations")
  public double getPositionVoltageSetpointRotations() {
    return positionVoltage.Position;
  }

  @Logged(name = "positionHoldSetpointRotations")
  public double getPositionHoldSetpointRotations() {
    return positionHold.Position;
  }

  /* Commands */

  public Command stop() {
    return runOnce(() -> motor.stopMotor()).withName(this.getName() + ".Stop");
  }

  private void controlMotor(Angle angle) {
    motor.setControl(motionMagicVoltage.withPosition(wrapDegreesToSoftLimits(angle)));
  }

  private static final double LARGE_JUMP_THRESHOLD = 0.2; // rotations (~72 degrees)

  private void controlMotor(Angle angle, AngularVelocity velocity) {
    Angle target = wrapDegreesToSoftLimits(angle);
    double errorRotations = Math.abs(motorPosition.getValue().minus(target).in(Rotations));

    if (errorRotations > LARGE_JUMP_THRESHOLD) {
      // wrap around or large repositioning - use motion magic for deceleration
      motor.setControl(motionMagicVoltage.withPosition(target));
    } else {
      Voltage velocityFeedForward = kV.times(velocity.in(RotationsPerSecond));

      motor.setControl(positionVoltage.withPosition(target).withFeedForward(velocityFeedForward));
    }
  }

  public Command setAngle(Angle angle) {
    return run(() -> controlMotor(angle)).withName(this.getName() + ".SetAngle");
  }

  public Command setAngle(Supplier<Angle> angle) {
    return run(() -> controlMotor(angle.get())).withName(this.getName() + ".SetAngleSupplier");
  }

  public Command setAngle(Supplier<Angle> angle, Supplier<AngularVelocity> velocity) {
    return run(() -> controlMotor(angle.get(), velocity.get()))
        .withName(this.getName() + ".SetAngleSupplierWithVelocity");
  }

  // set angle for turret relative to field element
  public Angle getAngleTo(Translation2d position) {
    return (AllianceFlipUtil.apply(
            position.minus(RobotState.getInstance().getRobotPose().getTranslation()).getAngle())
        .getMeasure()
        .minus(
            AllianceFlipUtil.apply(RobotState.getInstance().getRobotPose().getRotation())
                .getMeasure()));
  }

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
   * Call this in the turret's periodic() whenever the turret angle changes.
   *
   * @param turretAngleRads current turret angle in radians, field-positive CCW from robot forward
   */
  private void updateLimelightCameraPose(double turretAngleRads) {
    Limelight limelight = new Limelight("limelight");
    // 1. Robot origin → turret pivot (your 6.5" X/Y offsets, whatever height the pivot is at)
    Transform3d robotToTurretPivot =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(6.25), // forward from robot center
                Units.inchesToMeters(6.25), // left from robot center (adjust if needed)
                Units.inchesToMeters(24.867407) // height of turret pivot
                ),
            new Rotation3d(0, 0, 0) // no rotation yet — turret adds its own below
            );

    // 2. Turret pivot rotation (yaw only — this is what changes as the turret spins)
    Transform3d turretRotation =
        new Transform3d(new Translation3d(), new Rotation3d(0, 0, turretAngleRads));

    // 3. Turret pivot → camera (fixed mechanical offset of the Limelight on the turret)
    //    Adjust x/z/pitch to match your actual mount geometry
    // turret_center -> cameraLens = horizontal:  6.877436", vertical:  6.376978
    Transform3d turretToCamera =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(6.877436), // camera forward from turret pivot
                Units.inchesToMeters(0.0), // camera lateral offset
                Units.inchesToMeters(0.0) // camera height above pivot
                ),
            new Rotation3d(
                0,
                Units.degreesToRadians(20), // camera pitch (negative = tilted down)
                0));

    // 4. Chain: robot → pivot → (rotated by turret angle) → camera
    //    Pose3d.transformBy() accumulates transforms in order
    Pose3d cameraPoseRobotSpace =
        new Pose3d()
            .transformBy(robotToTurretPivot)
            .transformBy(turretRotation)
            .transformBy(turretToCamera);

    // 5. Push to YALL
    //  limelight.getSettings().withCameraOffset(cameraPoseRobotSpace);
  }

  /* Self Test */

  private boolean isNearTarget(Angle target) {
    return getAngle().isNear(target, SELF_TEST_VARIANCE_THRESHOLD);
  }

  // TODO: safeguard the position of the turret, should start at 0
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

  @Override
  public Command selfTestSlow() {
    return selfTestAt(Rotations.of(0.1), "Robot/Tests/turret/slow")
        .withName(getName() + ".SelfTestSlow");
  }

  @Override
  public Command selfTestFast() {
    return selfTestAt(Rotations.of(.25), "Robot/Tests/turret/fast")
        .withName(getName() + ".SelfTestFast");
  }

  /* Button Mappings for Copilot */

  public void buttonMappings() {
    Copilot.turret0Degrees().onTrue(setAngle(Degrees.of(0)));
    Copilot.turret45Degrees().onTrue(setAngle(Degrees.of(45)));
    Copilot.turret90Degrees().onTrue(setAngle(Degrees.of(90)));
    Copilot.turret135Degrees().onTrue(setAngle(Degrees.of(135)));
    Copilot.turret180Degrees().onTrue(setAngle(Degrees.of(180)));
    Copilot.turret225Degrees().onTrue(setAngle(Degrees.of(225)));
    Copilot.turret270Degrees().onTrue(setAngle(Degrees.of(270)));
    Copilot.turret315Degrees().onTrue(setAngle(Degrees.of(315)));
    Copilot.turretIdle().onTrue(setAngle(Degrees.of(360)));
  }
}
