
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.Supplier;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class TurretYAMS extends SubsystemBase {
  CANBus canbus = new CANBus("drive");
  private final TalonFX turretMotor = new TalonFX(30, canbus);

  private final SmartMotorControllerConfig motorConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withClosedLoopController(
              23.221, 0, .8981, RotationsPerSecond.of(20), RotationsPerSecondPerSecond.of(10))
          .withFeedforward(new SimpleMotorFeedforward(.38778, 2.3767, .077265))
          // Configure Motor and Mechanism properties
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(23.25))) // 12-30-10-93
          .withIdleMode(MotorMode.BRAKE)
          .withMotorInverted(false)
          // Setup Telemetry
          .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
          // Power Optimization
          .withStatorCurrentLimit(Amps.of(40))
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withOpenLoopRampRate(Seconds.of(0.25));
  private final SmartMotorController turretSMC =
      new TalonFXWrapper(turretMotor, DCMotor.getKrakenX44Foc(1), motorConfig);

  private final PivotConfig turretConfig =
      new PivotConfig(turretSMC)
          .withStartingPosition(Degrees.of(0)) // Starting position of the Pivot
          .withWrapping(
              Rotations.of(-.5), Rotations.of(.5)) // Wrapping enabled bc the pivot can spin
          // infinitely
          .withHardLimit(Degrees.of(-90), Degrees.of(90)) // Hard limit bc wiring prevents infinite
          // spinning
          .withTelemetry("TurretMech", TelemetryVerbosity.HIGH) // Telemetry
          .withMOI(Meters.of(0.25), Pounds.of(4)); // MOI Calculation

  private final Pivot turret = new Pivot(turretConfig);

  private final Supplier<Pose2d> poseSupplier;

  public TurretYAMS(Supplier<Pose2d> poseSupplier) {
    this.poseSupplier = poseSupplier;
  }

  Translation2d blueHub = new Translation2d(4.63, 4.03);
  @Logged Rotation2d robotRotation;
  @Logged Rotation2d turretRotation;
  @Logged Angle turretRelativeRotation;

  public Translation2d gethub() {
    return FlippingUtil.flipFieldPosition(blueHub);
  }

  // constantly gets the angle from the robot to the hub (turret rotation relative to hub)
  public void updateTurretRotation() {
    robotRotation = poseSupplier.get().getRotation();
    turretRotation = ((gethub().minus(poseSupplier.get().getTranslation())).getAngle());
    turretRelativeRotation = Degrees.of(robotRotation.minus(turretRotation).getDegrees());
  }

  @Logged
  public double getTurretRotationDegrees() {
    return turretRotation.getDegrees();
  }

  @Logged
  public double getRobotRotationDegrees() {
    return robotRotation.getDegrees();
  }

  @Logged
  public double getTurretRelativeRotationDegrees() {
    return turretRelativeRotation.in(Degrees);
  }

  @Logged
  public Translation2d getTargetPosition() {
    double distance =
        poseSupplier.get().getTranslation().getDistance(FlippingUtil.flipFieldPosition(blueHub));
    Transform2d targetTransform =
        new Transform2d(new Translation2d(distance, 0.0), Rotation2d.k180deg);

    Translation2d targetPos = poseSupplier.get().transformBy(targetTransform).getTranslation();
    return targetPos;
  }

  // turrets rotation relative to robot
  public Angle turretRelativeRotation() {
    return turretRelativeRotation;
  }

  // creates a new Pose2d that rotates around the hub
  @Logged
  public Pose2d getTurretPose() {
    return new Pose2d(poseSupplier.get().getTranslation(), turretRotation);
  }

  public boolean isAtAngle() {
    return turretSMC
        .getMechanismPosition()
        .isNear(turretSMC.getMechanismPositionSetpoint().orElse(Degrees.of(0)), Degrees.of(2.0));
  }

  public Command setAngle(Angle angle) {
    return turret.setAngle(angle);
  }

  public void setAngleDirect(Angle angle) {
    turretSMC.setPosition(angle);
  }

  public Command setAngle(Supplier<Angle> angleSupplier) {
    return turret.setAngle(angleSupplier);
  }

  public Angle getAngle() {
    return turret.getAngle();
  }

  public Command sysId() {
    return turret.sysId(
        Volts.of(4.0), // maximumVoltage
        Volts.per(Second).of(0.5), // step
        Seconds.of(8.0) // duration
        );
  }

  public Command setDutyCycle(Supplier<Double> dutyCycleSupplier) {
    return turret.set(dutyCycleSupplier);
  }

  public Command setDutyCycle(double dutyCycle) {
    return turret.set(dutyCycle);
  }

  @Override
  public void periodic() {
    updateTurretRotation();
    turret.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    turret.simIterate();
  }

  // SysId routines
  private final VoltageOut m_voltReq = new VoltageOut(0.0);

  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with Phoenix SignalLogger class
              (state) -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              (volts) -> turretMotor.setControl(m_voltReq.withOutput(volts.in(Volts))),
              null,
              this));

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}
