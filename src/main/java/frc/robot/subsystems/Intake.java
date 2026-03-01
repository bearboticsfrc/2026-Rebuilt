package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private final CANBus canivore = new CANBus("Default Name");
  private final CANBus rio = new CANBus("rio");

  private final TalonFX roller = new TalonFX(25, rio);
  private final TalonFX arm = new TalonFX(10, canivore);
  private final TalonFX mouth = new TalonFX(14, canivore);

  private final Angle extended = Degrees.of(20);
  private final Angle retratcted = Degrees.of(78);
  private Angle latestPosistion = retratcted;
  private final Angle oscillateMax = Degrees.of(60);
  private final Angle oscillateMin = Degrees.of(49);

  private final DutyCycleOut m_dutyReq = new DutyCycleOut(0.0);
  private final PositionVoltage m_posReq = new PositionVoltage(0.0);

  private final double MOUTH_SPEED = 0.3;
  private final double MOUTH_SLOW_SPEED = 0.15;
  public final double ROLLER_SPEED = 0.5;
  public final double GEAR_RATIO = 12;

  public Intake() {

    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    TalonFXConfiguration mouthConfig = new TalonFXConfiguration();
    TalonFXConfiguration armConfig = new TalonFXConfiguration();

    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    mouthConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    mouthConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    armConfig.Slot0.kP = 20;
    armConfig.Slot0.kG = 0;
    armConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    armConfig.Feedback.SensorToMechanismRatio = GEAR_RATIO;
    armConfig.Feedback.RotorToSensorRatio = 1.0;
    armConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

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

    arm.setPosition(retratcted.plus(Degrees.of(2)));
  }

  public void setRollerOutput(double output) {
    roller.setControl(m_dutyReq.withOutput(output));
  }

  public void setMouthOutput(double output) {
    mouth.setControl(m_dutyReq.withOutput(output));
  }

  public void setPosistion(Angle posistion) {
    arm.setControl(m_posReq.withPosition(posistion));
    latestPosistion = posistion;
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
    return runOnce(() -> setPosistion(extended));
  }

  public Command retractArm() {
    return runOnce(() -> setPosistion(retratcted));
  }

  public Command setArm(Angle angle) {
    return runOnce(() -> setPosistion(angle));
  }

  public Command intakeOut() {
    return extendArm()
        .andThen(
            Commands.waitUntil(() -> armIsExtended())
            .andThen(runRoller())
            .andThen(runMouth()));
  }

  public Command intakeIn() {
    return stopMouthCommand()
    .andThen(stopRollerCommand())
    .andThen(retractArm());
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
  public boolean armIsExtended() {
    return arm.getPosition()
    .getValue()
    .lt(extended.plus(Degrees.of(20)));
  }

  @Override
  public void periodic() {
    arm.setControl(m_posReq.withPosition(latestPosistion));
  }
}
