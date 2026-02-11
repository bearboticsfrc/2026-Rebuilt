package frc.robot.subsystems.Turret;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.spectrumLib.CachedDouble;
import frc.spectrumLib.util.Conversions;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;

public class Turret extends SubsystemBase implements NTSendable {

  private final CANBus canivore = new CANBus("drive");

  private final TalonFX motor = new TalonFX(30, canivore);

  PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(0);
  MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

  /* Turret config values */
  @Getter private double currentLimit = 44;
  @Getter private double torqueCurrentLimit = 200;
  @Getter private double velocityKp = 23.221;
  @Getter private double velocityKi = 0.4; // try lowering this
  @Getter private double velocityKd = 0.8981;

  @Getter private double velocityKs = .38778 * 2.0;
  @Getter private double velocityKv = 2.3767;
  @Getter private double velocityKa = .077265;

  @Getter private double gearRatio = 23.25;

  @Getter private double statorLimit = 40;

  private final CachedDouble cachedRotations;
  private final CachedDouble cachedVoltage;
  private final CachedDouble cachedDegrees;
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

  @Setter @Getter private boolean active;

  public Turret() {

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.Slot0.kP = velocityKp;
    config.Slot0.kI = velocityKi;
    config.Slot0.kD = velocityKd;

    config.Slot0.kA = velocityKa;
    config.Slot0.kS = velocityKs;
    config.Slot0.kV = velocityKv;

    var motionMagicConfigs = config.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration =
        320; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 1280; // Target jerk of 1600 rps/s/s (0.1 seconds)

    config.Feedback.RotorToSensorRatio = 1.0;
    config.Feedback.SensorToMechanismRatio = gearRatio;

    config.CurrentLimits.StatorCurrentLimit = statorLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    StatusCode status = motor.getConfigurator().apply(config);

    for (int i = 0; i < 2; i++) {
      if (status.isOK()) break;
      status = motor.getConfigurator().apply(config);
    }

    cachedCurrent = new CachedDouble(this::updateCurrent);
    cachedVoltage = new CachedDouble(this::updateVoltage);
    cachedRotations = new CachedDouble(this::updatePositionRotations);
    cachedDegrees = new CachedDouble(this::updatePositionDegrees);
    cachedVelocity = new CachedDouble(this::updateVelocityRPM);

    if (!status.isOK()) {
      System.out.println("ERROR Configuring Turret motor: " + status);
    }

    System.out.println("Turret Subsystem Initialized");

    System.out.println("MotionMagic:" + motionMagicVoltage);
  }

  @Override
  public void periodic() {
    if (isActive()) {}
    updateCurrent();
    updatePositionDegrees();
    updatePositionRotations();
    updateVelocityRPM();
    updateVelocityRPS();
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
    return cachedRotations.getAsDouble() * 360.0;
  }

  public double getVelocityRPM() {
    return cachedVelocity.getAsDouble();
  }

  @Logged
  public double getStatorCurrent() {
    return cachedCurrent.getAsDouble();
  }

  @Logged
  public double updatePositionDegrees() {
    return rotationsToDegrees(this::getPositionRotations);
  }

  @Logged
  public double getSetpoint() {
    return motionMagicVoltage.Position * 360.0;
  }

  @Logged
  public double getSetpointRotations() {
    return motionMagicVoltage.Position;
  }

  /**
   * Rotations to Degrees
   *
   * @param rotations
   * @return degrees
   */
  public double rotationsToDegrees(DoubleSupplier rotations) {
    return 360 * rotations.getAsDouble();
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

  public Command setAngle(Angle angle) {
    return Commands.run(() -> motor.setControl(motionMagicVoltage.withPosition(angle)), this)
        .withName(this.getName() + " SetAngle");
  }

  public Command setAngle(Supplier<Angle> angle) {
    return Commands.run(() -> motor.setControl(motionMagicVoltage.withPosition(angle.get())), this)
        .withName(this.getName() + " SetAngleSupplier");
  }
}
