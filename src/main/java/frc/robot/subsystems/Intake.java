package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;

public class Intake extends SubsystemBase {
  /** Position setpoints for the Intake Arm. */
  public enum Setpoint {
    Initial(Degrees.of(80)),

    Retracted(Degrees.of(78)),
    AlmostExtended(Degrees.of(40)),
    Extended(Degrees.of(22));

    /** The position target of the setpoint in angular units. */
    public final Angle target;

    private Setpoint(Angle target) {
      this.target = target;
    }
  }

  private final CANBus canivore = new CANBus("Default Name");
  private final CANBus rio = new CANBus("rio");

  private final TalonFX roller = new TalonFX(25, rio);
  private final TalonFX arm = new TalonFX(10, canivore);
  private final TalonFX mouth = new TalonFX(14, canivore);

  private final DutyCycleOut dutyReq = new DutyCycleOut(0.0);
  private final PositionVoltage posReq = new PositionVoltage(0.0);
  private final MotionMagicVoltage motionMagicVoltageRequest = new MotionMagicVoltage(0);

  /* device status signals */
  private final StatusSignal<Angle> armPosition = arm.getPosition(false);
  private final StatusSignal<AngularVelocity> armVelocity = arm.getVelocity(false);
  private final StatusSignal<Current> armTorqueCurrent = arm.getTorqueCurrent(false);

  private final double MOUTH_SPEED = -0.3;
  private final double MOUTH_SLOW_SPEED = -0.15;
  public final double ROLLER_SPEED = 0.5;
  public final double ARM_GEAR_RATIO = 12;
  private Angle latestPosistion = Setpoint.Retracted.target;

  /** Configs common across all motors. */
  private static final TalonFXConfiguration motorInitialConfigs = new TalonFXConfiguration();

  /** Configs common across just the leader motors. */
  private static final TalonFXConfiguration armInitialConfigs = motorInitialConfigs.clone();

  /** Configs for arm */
  private final TalonFXConfiguration armConfig =
      armInitialConfigs
          .clone()
          .withMotorOutput(
              armInitialConfigs
                  .MotorOutput
                  .clone()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.CounterClockwise_Positive))
          .withCurrentLimits(
              armInitialConfigs
                  .CurrentLimits
                  .clone()
                  .withStatorCurrentLimit(Amps.of(120))
                  .withStatorCurrentLimitEnable(true))
          .withSlot0(
              armInitialConfigs
                  .Slot0
                  .clone()
                  .withKP(20)
                  .withKD(0)
                  .withKS(0)
                  .withKG(0)
                  .withGravityType(GravityTypeValue.Arm_Cosine)
                  .withGravityArmPositionOffset(Degrees.of(20)))
          .withFeedback(
              armInitialConfigs.Feedback.clone().withSensorToMechanismRatio(ARM_GEAR_RATIO))
          .withMotionMagic(
              armInitialConfigs
                  .MotionMagic
                  .clone()
                  .withMotionMagicCruiseVelocity(RotationsPerSecond.of(200))
                  .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(600)));

  public Intake() {

    TalonFXConfiguration rotationConfig = new TalonFXConfiguration();
    //  TalonFXConfiguration armConfig = new TalonFXConfiguration();

    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // armConfig.Slot0.kP = 20;
    // armConfig.Slot0.kG = 0;
    // armConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    // armConfig.Feedback.SensorToMechanismRatio = GEAR_RATIO;
    // armConfig.Feedback.RotorToSensorRatio = 1.0;
    // armConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    for (int i = 0; i < 2; ++i) {
      var status = roller.getConfigurator().apply(rollerConfig);
      if (status.isOK()) break;
    }

    for (int i = 0; i < 2; ++i) {
      var status = arm.getConfigurator().apply(armConfig);
      if (status.isOK()) break;
    }

    for (int i = 0; i < 2; ++i) {
      var status = mouth.getConfigurator().apply(mouthConfig);
      if (status.isOK()) break;
    }

    arm.setPosition(Setpoint.Initial.target);
  }

  public void setRollerOutput(double output) {
    roller.setControl(dutyReq.withOutput(output));
  }

  public void setMouthOutput(double output) {
    mouth.setControl(dutyReq.withOutput(output));
  }

  public void setPosistion(Setpoint setpoint) {
    arm.setControl(posReq.withPosition(setpoint.target));
    latestPosistion = setpoint.target;
  }

  public void stopRoller() {
    roller.stopMotor();
  }

  public void stopMouth() {
    mouth.stopMotor();
  }

  public Command runRoller() {
    return runOnce(() -> setRollerOutput(ROLLER_SPEED));
  }

  public Command runMouth() {
    return runOnce(() -> setMouthOutput(MOUTH_SPEED));
  }

  public Command runMouthSlow() {
    return runOnce(() -> setMouthOutput(MOUTH_SLOW_SPEED));
  }

  public Command stopMouthCommand() {
    return runOnce(() -> stopMouth());
  }

    public Command stopRollerCommand() {
    return runOnce(() -> stopRoller());
  }

  public Command extendArm() {
    return goToSetpoint(() -> Setpoint.Extended);
    // return runOnce(() -> setPosistion(Setpoint.Extended));
  }

  public Command retractArm() {
    // return goToSetpoint(() -> Setpoint.Retracted);

    return runOnce(() -> setPosistion(Setpoint.Retracted));
  }

  public Command setArm(Angle angle) {
    return runOnce(() -> setPosistion(angle));
  }

  public Command intakeOut() {
    return runRoller().andThen(runMouth()).andThen(extendArm());
  }

  public Command intakeIn() {
    return retractArm().andThen(stopRollerCommand()).andThen(stopMouthCommand());
    // return stopMouthCommand().andThen(stopRollerCommand()).andThen(retractArm());
  }

  public Command armOscillate(){
    return setArm(oscillateMin)
    .andThen(Commands.waitSeconds(.75))
    .andThen(setArm(oscillateMax));
  }

  @Logged
  public double getRollerVelocityInRPM() {
    return roller.getVelocity().getValue().in(RPM);
  }

  @Logged
  public double getMouthVelocityInRPM() {
    return roller.getVelocity().getValue().in(RPM);
  }

  @Logged
  public double getArmPosistion() {
    return arm.getPosition().getValue().in(Degrees);
  }

  @Logged
  public double getArmSetpoint() {
    return m_posReq.getPositionMeasure().in(Degrees);
  }

  @Logged
  public double getArmVoltage() {
    return arm.getMotorVoltage().getValueAsDouble();
  }

  @Logged
  public double getArmSetpoint() {
    return motionMagicVoltageRequest.getPositionMeasure().in(Degrees);
    // return posReq.getPositionMeasure().in(Degrees);
  }

  @Logged
  public boolean armIsExtended() {
    return arm.getPosition().getValue().lt(Setpoint.AlmostExtended.target);
  }

  /**
   * Holds the hood at the current position using PID.
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
   * Drives the hood to the provided position setpoint.
   *
   * @param setpoint Function returning the setpoint to apply
   * @return Command to run
   */
  public Command goToSetpoint(Supplier<Setpoint> setpoint) {
    return run(
        () -> {
          motionMagicVoltageRequest.withPosition(setpoint.get().target);
          arm.setControl(motionMagicVoltageRequest);
        });
  }

  @Override
  public void periodic() {
    /* refresh all status signals */
    BaseStatusSignal.refreshAll(armPosition, armVelocity, armTorqueCurrent);

    // arm.setControl(motionMagicVoltageRequest.withPosition(latestPosistion));

    // arm.setControl(posReq.withPosition(latestPosistion));
  }
}
