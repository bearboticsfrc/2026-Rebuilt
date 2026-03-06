package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.field.AllianceFlipUtil;
import frc.spectrumLib.CachedDouble;
import frc.spectrumLib.util.Conversions;
import java.util.function.Supplier;
import lombok.Getter;

public class Turret extends SubsystemBase implements NTSendable {

  private final CANBus canivore = new CANBus("Default Name");

  private final TalonFX motor = new TalonFX(22, canivore);

  // MotionMagicTorqueCurrentFOC motionMagicTorqueCurrentFOC = new MotionMagicTorqueCurrentFOC(0);
  private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);
  private PositionVoltage positionHold = new PositionVoltage(0).withSlot(1);

  /* Turret config values */
  @Getter private double currentLimit = 44;
  @Getter private double torqueCurrentLimit = 400;

  @Getter private double gearRatio = 10.44; //  4.35;

  @Getter private double statorLimit = 200;

  private final CachedDouble cachedRotations;
  private final CachedDouble cachedVoltage;
  private final CachedDouble cachedVelocity;
  private final CachedDouble cachedCurrent;

  //   public TurretConfig() {
  //     super("Turret", 5, Rio.CANIVORE);
  //     configPIDGains(0, velocityKp, 0, 0);
  //     configFeedForwardGains(velocityKs, velocityKv, 0, 0);
  //     configGearRatio(1);
  //     configSupplyCurrentLimit(currentLimit, true);
  //     configStatorCurrentLimit(torqueCurrentLimit, true);
  //     configForwardTorqueCurrentLimit(torqueCurrentLimit);
  //     configReverseTorqueCurrentLimit(torqueCurrentLimit);
  //     configNeutralBrakeMode(true);
  //     configCounterClockwise_Positive();
  //     setAttached(true);
  //   }
  // }

  // set these small to start
  @Getter public Angle minRotations = Rotations.of(-.4);
  @Getter public Angle maxRotations = Rotations.of(.35);

  public Turret() {
    super("Turret");

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.Slot0.kP =
        70; // 23.221; //   72 for 5 degrees, 180 for 2 degrees, 360 for 1 degrees.  Increase D
    // with
    // increase in P
    config.Slot0.kD = 3; // .8981; //   start with d = p / 100 and increase until oscillation stops

    config.Slot0.kA = 0; // .077265;
    config.Slot0.kS = .6; // .2; // start with .4; tune up if mechanism stalls at end of moves
    config.Slot0.kV = 0.54; // ( 0.124 x 4.34 = .54  V/mechanism-RPS )

    config.Slot1.kP = 150;
    config.Slot1.kD = 12;
    config.Slot1.kA = 0;
    config.Slot1.kS = 0;
    config.Slot1.kV = 0;

    var motionMagicConfigs = config.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 1.8; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration =
        3.6; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 25; // Target jerk of 1600 rps/s/s (0.1 seconds)

    config.Feedback.RotorToSensorRatio = 1.0;
    config.Feedback.SensorToMechanismRatio = gearRatio;

    config.CurrentLimits.SupplyCurrentLimit = 60;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = statorLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.TorqueCurrent.PeakForwardTorqueCurrent = 120; // amps -- tune up from here
    config.TorqueCurrent.PeakReverseTorqueCurrent = -120;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = .35;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -.4;

    // motionMagicVoltage.withFeedForward(Volts.of(1));

    StatusCode status = motor.getConfigurator().apply(config);

    for (int i = 0; i < 2; i++) {
      if (status.isOK()) break;
      status = motor.getConfigurator().apply(config);
    }

    motor.setPosition(0);

    cachedCurrent = new CachedDouble(this::updateCurrent);
    cachedVoltage = new CachedDouble(this::updateVoltage);
    cachedRotations = new CachedDouble(this::updatePositionRotations);
    cachedVelocity = new CachedDouble(this::updateVelocityRPM);

    if (!status.isOK()) {
      System.out.println("ERROR Configuring Turret motor: " + status);
    }

    optimizeCAN();

    System.out.println("Turret Subsystem Initialized");
  }

  private void optimizeCAN() {
    motor.getPosition().setUpdateFrequency(100);
    motor.getVelocity().setUpdateFrequency(100);
    motor.getSupplyCurrent().setUpdateFrequency(50);
    motor.getDeviceTemp().setUpdateFrequency(4);
    motor.getClosedLoopReference().setUpdateFrequency(100);

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
  public void periodic() {
    updateCurrent();
    updatePositionRotations();
    updateVelocityRPM();
    updateVoltage();
  }

  @Override
  public void initSendable(NTSendableBuilder builder) {
    builder.addStringProperty("CurrentCommand", this::getCurrentCommandName, null);
    builder.addDoubleProperty("Motor Voltage", this::getVoltage, null);
    builder.addDoubleProperty("Rotations", this::getPositionRotations, null);
    builder.addDoubleProperty("setPoint", this::getSetpointRotations, null);
    builder.addDoubleProperty("Velocity RPM", this::getVelocityRPM, null);
    builder.addDoubleProperty("StatorCurrent", this::getStatorCurrent, null);
  }

  protected String getCurrentCommandName() {
    Command currentCommand = this.getCurrentCommand();
    if (currentCommand != null) {
      return currentCommand.getName();
    }

    return "none";
  }

  @Logged
  public double getVoltage() {
    return cachedVoltage.getAsDouble();
  }

  public double updateVoltage() {
    return motor.getMotorVoltage().getValueAsDouble();
  }

  /**
   * Update the value of the stator current for the motor
   *
   * @return
   */
  public double updateCurrent() {
    return motor.getStatorCurrent().getValueAsDouble();
  }

  /**
   * Updates the position of the motor
   *
   * @return motor position in rotations
   */
  private double updatePositionRotations() {
    return motor.getPosition().getValueAsDouble();
  }

  @Logged
  public double getPositionRotations() {
    return cachedRotations.getAsDouble();
  }

  @Logged
  public Angle getAngle() {
    return motor.getPosition().getValue();
  }

  public double getVelocityRPM() {
    return cachedVelocity.getAsDouble();
  }

  @Logged
  public double getStatorCurrent() {
    return cachedCurrent.getAsDouble();
  }

  @Logged
  public double getSetpoint() {
    return motor.getClosedLoopReference().getValueAsDouble() * 360.0;
  }

  @Logged
  public double getSetpointRotations() {
    return motor.getClosedLoopReference().getValueAsDouble();
  }

  @Logged
  public double getMotionMagicSetpointRotations() {
    return motionMagicVoltage.Position;
  }

  @Logged
  public double getPositionHoldSetpointRotations() {
    return positionHold.Position;
  }

  // Get Velocity in RPM
  private double updateVelocityRPM() {
    return Conversions.RPStoRPM(updateVelocityRPS());
  }

  /**
   * Updates the velocity of the motor
   *
   * @return motor velocity in rotations/sec which are the CTRE native units
   */
  private double updateVelocityRPS() {
    return motor.getVelocity().getValueAsDouble();
  }

  public Command stop() {
    return Commands.runOnce(() -> motor.stopMotor());
  }

  private void controlMotor(Angle angle) {
    // double errorDeg = Math.abs(motor.getPosition().getValue().minus(angle).in(Degrees));
    // if (errorDeg > 2.0) {
    motor.setControl(motionMagicVoltage.withPosition(wrapDegreesToSoftLimits(angle)));
    // } else {
    //  motor.setControl(positionHold.withPosition(wrapDegreesToSoftLimits(angle)));
    // }
  }

  public Command setAngle(Angle angle) {
    return Commands.run(() -> controlMotor(angle), this).withName(this.getName() + " SetAngle");
  }

  public Command setAngle(Supplier<Angle> angle) {
    return Commands.run(() -> controlMotor(angle.get()), this)
        .withName(this.getName() + " SetAngleSupplier");
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
            new Translation2d(RobotState.getInstance().getDistanceToHub(), 0.0),
            Rotation2d.k180deg);

    Pose2d turretPose =
        RobotState.getInstance()
            .robotPose
            .transformBy(RobotState.turretToRobot)
            .transformBy(new Transform2d(0, 0, new Rotation2d(getAngle())));

    Translation2d targetPos = turretPose.transformBy(targetTransform).getTranslation();
    return targetPos;
  }
}
