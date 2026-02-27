package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase {
  
  private final CANBus canivore = new CANBus("drive");

  private final TalonFX flywheel = new TalonFX(14, canivore);
  private final TalonFX arm = new TalonFX(10, canivore);

  private final double extended = 0.0;
  private final double retratcted = 0.0;

  private final VoltageOut m_voltReq = new VoltageOut(0.0);
  private final PositionVoltage m_posReq = new PositionVoltage(0.0);

  public Intake() {

    TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
    TalonFXConfiguration armConfig = new TalonFXConfiguration();

    flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    flywheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    flywheelConfig.Slot0.kS = 0;
    flywheelConfig.Slot0.kV = 0;
    flywheelConfig.Slot0.kA = 0;
    flywheelConfig.Slot0.kP = 0;

    armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    flywheelConfig.Slot0.kP = 0;
    flywheelConfig.Slot0.kI = 0;
    flywheelConfig.Slot0.kD = 0;
    flywheelConfig.Slot0.kG = 0;

    for (int i = 0; i < 2; ++i) {
      var status = flywheel.getConfigurator().apply(flywheelConfig);
      if (status.isOK()) break;
    }
  
    for (int i = 0; i < 2; ++i) {
      var status = arm.getConfigurator().apply(armConfig);
      if (status.isOK()) break;
    }
  }

  public void setOutput(double output){
    flywheel.setControl(m_voltReq.withOutput(output));
  }

  public void setPosistion(double posistion){
    arm.setControl(m_posReq.withPosition(posistion));
  }

  public void stopFlywheel(){
    flywheel.stopMotor();
  }

  public Command runFlywheel(){
    return runOnce(()-> setOutput(0));
  }

  public Command extendArm(){
    return runOnce(()-> setPosistion(extended));
  }

  public Command retractArm(){
    return runOnce(()-> setPosistion(retratcted));
  }

  public Command stopFlywheelCommand(){
    return runOnce(()-> stopFlywheel());
  }

  public Command intakeOut(){
    return runOnce(()-> extendArm().andThen(runFlywheel()));
  }

  public Command intakeIn(){
    return runOnce(()-> stopFlywheelCommand().andThen(retractArm()));
  }

  @Logged
  public double getFlywheelVelocityInRPM() {
    return flywheel.getVelocity().getValue().in(RPM);
  }

  @Logged
  public AngularVelocity getFlywheelVelocity() {
    // Get the current velocity of the flywheel
    return flywheel.getVelocity().getValue();
  }

  @Logged
  public Angle getArmPosistion(){
    return arm.getPosition().getValue();
  }

  public Angle getTargetPose() {
    return m_posReq.getPositionMeasure();
  }

  @Logged
  public boolean isAtTargetPose(){
    return getArmPosistion().equals(getArmPosistion());
  }
}
  
