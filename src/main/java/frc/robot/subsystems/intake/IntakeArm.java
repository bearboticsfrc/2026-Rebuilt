package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;

public class IntakeArm extends SubsystemBase {
  /** Position setpoints for the Intake Arm. */
  public enum Setpoint {
    Initial(Degrees.of(80)),

    Retracted(Degrees.of(78)),
    OscillateMin(Degrees.of(60)),
    OscillateMax(Degrees.of(30)),

    AlmostExtended(Degrees.of(40)),
    Extended(Degrees.of(22));

    /** The position target of the setpoint in angular units. */
    public final Angle target;

    private Setpoint(Angle target) {
      this.target = target;
    }
  }

  private final CANBus canivore = new CANBus("Default Name");

  private final TalonFX motor = new TalonFX(10, canivore);

  private final MotionMagicVoltage motionMagicVoltageRequest = new MotionMagicVoltage(0);

  /* device status signals */
  private final StatusSignal<Angle> armPosition = motor.getPosition(false);
  private final StatusSignal<AngularVelocity> armVelocity = motor.getVelocity(false);
  private final StatusSignal<Current> armTorqueCurrent = motor.getTorqueCurrent(false);
  private final StatusSignal<Voltage> armVoltage = motor.getMotorVoltage(false);

  private final StatusSignal<Double> armProfileVelocity = motor.getClosedLoopReferenceSlope(false);

  private final double ARM_GEAR_RATIO = 12;

  /** Configs common across all motors. */
  private static final TalonFXConfiguration motorInitialConfigs = new TalonFXConfiguration();

  /** Configs for arm */
  private final TalonFXConfiguration config =
      motorInitialConfigs
          .clone()
          .withMotorOutput(
              motorInitialConfigs
                  .MotorOutput
                  .clone()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.CounterClockwise_Positive))
          .withCurrentLimits(
              motorInitialConfigs
                  .CurrentLimits
                  .clone()
                  .withStatorCurrentLimit(Amps.of(200))
                  .withStatorCurrentLimitEnable(true))
          .withSlot0(
              motorInitialConfigs
                  .Slot0
                  .clone()
                  .withKP(100)
                  .withKD(2)
                  .withKS(.3)
                  .withKG(0)
                  .withGravityType(GravityTypeValue.Arm_Cosine)
                  .withGravityArmPositionOffset(Degrees.of(0)))
          .withFeedback(
              motorInitialConfigs.Feedback.clone().withSensorToMechanismRatio(ARM_GEAR_RATIO))
          .withMotionMagic(
              motorInitialConfigs
                  .MotionMagic
                  .clone()
                  .withMotionMagicCruiseVelocity(RotationsPerSecond.of(.5))
                  .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(2.0))
                  .withMotionMagicJerk(5.0));

  /*
   * try:
   * kp=150
   * kd=2
   * ks=.3
   *          .9 sec  / .5 sec / .25 sec
   * mm_cruise= 0.3   or 0.5  or 1.0
   * mm_accel = 1.0   0r 2.0  or 5.0
   *
   * Use jerk = 5.0
   *
   * chech stator current limit ... increase from 120 ???
   */

  public IntakeArm() {
    super("IntakeArm");

    StatusCode status = motor.getConfigurator().apply(config);

    for (int i = 0; i < 2; ++i) {
      if (status.isOK()) break;
      status = motor.getConfigurator().apply(config);
    }
    if (!status.isOK()) {
      DriverStation.reportError("ERROR Configuring IntakeArm arm motor: " + status, false);
    }

    motor.setPosition(Setpoint.Initial.target);

    optimizeCAN();
    System.out.println("IntakeArm Subsystem Initialized");
  }

  private void optimizeCAN() {
    motor.getPosition().setUpdateFrequency(100);
    motor.getVelocity().setUpdateFrequency(100);
    motor.getSupplyCurrent().setUpdateFrequency(50);
    motor.getDeviceTemp().setUpdateFrequency(10);

    motor.optimizeBusUtilization();
  }

  public Command extend() {
    return goToSetpoint(() -> Setpoint.Extended);
  }

  public Command retract() {
    return goToSetpoint(() -> Setpoint.Retracted);
  }

  public Command armOscillate() {
    return goToSetpoint(() -> Setpoint.OscillateMin)
        .withTimeout(.75)
        .andThen(goToSetpoint(() -> Setpoint.OscillateMax).withTimeout(.75))
        .repeatedly();
  }

  @Logged
  public double getArmPosition() {
    return armPosition.getValue().in(Degrees);
  }

  @Logged
  public double getProvileVelocityRPS() {
    return armProfileVelocity.getValue();
  }

  @Logged
  public double getArmSetpoint() {
    return motionMagicVoltageRequest.getPositionMeasure().in(Degrees);
  }

  @Logged
  public double getArmVoltage() {
    return armVoltage.getValueAsDouble();
  }

  @Logged
  public boolean armIsExtended() {
    return armPosition.getValue().lt(Setpoint.AlmostExtended.target);
  }

  /**
   * Holds the arm at the current position using PID.
   *
   * @return Command to run
   */
  public Command holdPosition() {
    return runOnce(() -> motionMagicVoltageRequest.withPosition(armPosition.getValue()))
        .andThen(
            run(
                () -> {
                  motor.setControl(motionMagicVoltageRequest);
                }));
  }

  /**
   * Drives the arm to the provided position setpoint.
   *
   * @param setpoint Function returning the setpoint to apply
   * @return Command to run
   */
  private Command goToSetpoint(Supplier<Setpoint> setpoint) {
    return run(
        () -> {
          motionMagicVoltageRequest.withPosition(setpoint.get().target);
          motor.setControl(motionMagicVoltageRequest);
        });
  }

  @Override
  public void periodic() {
    /* refresh all status signals */
    BaseStatusSignal.refreshAll(
        armPosition, armVelocity, armTorqueCurrent, armVoltage, armProfileVelocity);
  }
}
