package frc.robot.subsystems;

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

  private final TalonFX arm = new TalonFX(10, canivore);

  private final MotionMagicVoltage motionMagicVoltageRequest = new MotionMagicVoltage(0);

  /* device status signals */
  private final StatusSignal<Angle> armPosition = arm.getPosition(false);
  private final StatusSignal<AngularVelocity> armVelocity = arm.getVelocity(false);
  private final StatusSignal<Voltage> armVoltage = arm.getMotorVoltage(false);

  private final StatusSignal<Double> armProfileVelocity = arm.getClosedLoopReferenceSlope(false);

  private final double ARM_GEAR_RATIO = 12;

  /** Configs common across all motors. */
  private static final TalonFXConfiguration motorInitialConfigs = new TalonFXConfiguration();

  /** Configs for arm */
  private final TalonFXConfiguration armConfig =
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
                  .withSupplyCurrentLimit(Amps.of(40))
                  .withSupplyCurrentLimitEnable(true)
                  .withStatorCurrentLimit(Amps.of(120))
                  .withStatorCurrentLimitEnable(true))
          .withSlot0(
              motorInitialConfigs
                  .Slot0
                  .clone()
                  .withKP(20)
                  .withKD(0)
                  .withKS(0)
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

    StatusCode status = arm.getConfigurator().apply(armConfig);

    for (int i = 0; i < 2; ++i) {
      if (status.isOK()) break;
      status = arm.getConfigurator().apply(armConfig);
    }
    if (!status.isOK()) {
      DriverStation.reportError("ERROR Configuring IntakeArm arm motor: " + status, false);
    }

    arm.setPosition(Setpoint.Initial.target);

    optimizeCAN();
    System.out.println("IntakeArm Subsystem Initialized");
  }

  private void optimizeCAN() {
    arm.getPosition().setUpdateFrequency(100);
    arm.getVelocity().setUpdateFrequency(100);
    arm.getSupplyCurrent().setUpdateFrequency(50);
    arm.getDeviceTemp().setUpdateFrequency(10);

    arm.optimizeBusUtilization();
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
  public double getProfileVelocityRPS() {
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
                  arm.setControl(motionMagicVoltageRequest);
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
          arm.setControl(motionMagicVoltageRequest);
        });
  }

  @Override
  public void periodic() {
    /* refresh all status signals */
    BaseStatusSignal.refreshAll(armPosition, armVelocity, armVoltage, armProfileVelocity);
  }
}
