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

  private final TalonFX rollers = new TalonFX(25, rio);
  private final TalonFX arm = new TalonFX(10, canivore);
  private final TalonFX mouth = new TalonFX(14, canivore);

  private final Angle extended = Degrees.of(20); // -1.8
  private final Angle retratcted = Degrees.of(78);
  private final DutyCycleOut m_dutyReq = new DutyCycleOut(0.0);
  private final PositionVoltage m_posReq = new PositionVoltage(0.0);
  private final double MOUTHSPEED = -0.3;
  public final double ROLLERSPEED = 0.5;
  public final double gearRatio = 12;

  public Intake() {

    TalonFXConfiguration rollersConfig = new TalonFXConfiguration();
    TalonFXConfiguration armConfig = new TalonFXConfiguration();

    rollersConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rollersConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rollersConfig.Slot0.kS = 0;
    rollersConfig.Slot0.kV = 0;
    rollersConfig.Slot0.kA = 0;
    rollersConfig.Slot0.kP = 0;

    armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    armConfig.Slot0.kP = 20;
    armConfig.Slot0.kG = 0;
    armConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    armConfig.Feedback.SensorToMechanismRatio = gearRatio;
    armConfig.Feedback.RotorToSensorRatio = 1.0;
    armConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    for (int i = 0; i < 2; ++i) {
      var status = rollers.getConfigurator().apply(rollersConfig);
      if (status.isOK()) break;
    }

    for (int i = 0; i < 2; ++i) {
      var status = arm.getConfigurator().apply(armConfig);
      if (status.isOK()) break;
    }

    for (int i = 0; i < 2; ++i) {
      var status = mouth.getConfigurator().apply(rollersConfig);
      if (status.isOK()) break;
    }

    arm.setPosition(retratcted.plus(Degrees.of(2)));
  }

  public void setOutput(double output) {
    rollers.setControl(m_dutyReq.withOutput(output));
  }

  public void setMouthOutput(double output) {
    mouth.setControl(m_dutyReq.withOutput(output));
  }

  public void setPosistion(Angle posistion) {
    arm.setControl(m_posReq.withPosition(posistion));
  }

  public void stopRollers() {
    rollers.stopMotor();
  }

  public void stopMouth() {
    mouth.stopMotor();
  }

  @Logged
  public boolean armIsExtended() {
    return arm.getPosition().getValue().lt(extended.plus(Degrees.of(20)));
  }

  public Command runRollers() {
    return runOnce(() -> setOutput(ROLLERSPEED));
  }

  public Command runMouth() {
    return runOnce(() -> setMouthOutput(MOUTHSPEED));
  }

  public Command stopMouthCommand() {
    return runOnce(() -> stopMouth());
  }

  public Command extendArm() {
    return runOnce(() -> setPosistion(extended));
  }

  public Command retractArm() {
    return runOnce(() -> setPosistion(retratcted));
  }

  public Command stopRollersCommand() {
    return runOnce(() -> stopRollers());
  }

  public Command intakeOut() {
    return extendArm()
        .andThen(
            Commands.waitUntil(() -> armIsExtended()).andThen(runRollers()).andThen(runMouth()));
  }

  public Command intakeIn() {
    return stopMouthCommand().andThen(stopRollersCommand()).andThen(retractArm());
  }

  @Logged
  public double getRollersyInRPM() {
    return rollers.getVelocity().getValue().in(RPM);
  }

  @Logged
  public AngularVelocity getFlywheelVelocity() {
    // Get the current velocity of the flywheel
    return rollers.getVelocity().getValue();
  }

  @Logged
  public double getArmPosistion() {
    return arm.getPosition().getValue().in(Degrees);
  }

  @Logged
  public double getArmVoltage() {
    return arm.getMotorVoltage().getValueAsDouble();
  }

  @Logged
  public double getArmSetpoint() {
    return m_posReq.getPositionMeasure().in(Degrees);
  }
}
