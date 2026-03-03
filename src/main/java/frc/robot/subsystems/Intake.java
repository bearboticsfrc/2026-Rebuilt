package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private final CANBus canivore = new CANBus("Default Name");
  private final CANBus rio = new CANBus("rio");

  private final TalonFX roller = new TalonFX(25, rio);
  private final TalonFX mouth = new TalonFX(14, canivore);

  private final DutyCycleOut dutyReq = new DutyCycleOut(0.0);

  /* device status signals */
  // private final StatusSignal<AngularVelocity> rollerVelocity = roller.getVelocity(false);
  private final StatusSignal<AngularVelocity> mouthVelocity = mouth.getVelocity(false);

  private final double MOUTH_SPEED = 0.3;
  private final double MOUTH_SLOW_SPEED = 0.15;
  public final double ROLLER_SPEED = 0.5;

  public Intake() {
    super("Intake");
    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    TalonFXConfiguration mouthConfig = new TalonFXConfiguration();

    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    mouthConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    mouthConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    StatusCode status = roller.getConfigurator().apply(rollerConfig);

    for (int i = 0; i < 2; ++i) {
      if (status.isOK()) break;
      status = roller.getConfigurator().apply(rollerConfig);
    }
    if (!status.isOK()) {
      System.out.println("ERROR Configuring Intake roller motor: " + status);
    }

    status = mouth.getConfigurator().apply(mouthConfig);

    for (int i = 0; i < 2; ++i) {
      if (status.isOK()) break;
      status = mouth.getConfigurator().apply(mouthConfig);
    }
    if (!status.isOK()) {
      System.out.println("ERROR Configuring Intake mouth motor: " + status);
    }

    optimizeCAN();
    System.out.println("Intake Subsystem Initialized");
  }

  private void optimizeCAN() {
    // roller.getPosition().setUpdateFrequency(50);
    // roller.getVelocity().setUpdateFrequency(50);
    // roller.getSupplyCurrent().setUpdateFrequency(50);
    // roller.getDeviceTemp().setUpdateFrequency(10);

    // roller.optimizeBusUtilization();

    mouth.getPosition().setUpdateFrequency(50);
    mouth.getVelocity().setUpdateFrequency(50);
    mouth.getSupplyCurrent().setUpdateFrequency(50);
    mouth.getDeviceTemp().setUpdateFrequency(10);

    mouth.optimizeBusUtilization();
  }

  public void setRollerOutput(double output) {
    roller.setControl(dutyReq.withOutput(output));
  }

  public void setMouthOutput(double output) {
    mouth.setControl(dutyReq.withOutput(output));
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

  @Logged
  public double getRollerVelocityInRPM() {
    return roller.getVelocity().getValue().in(RPM);
  }

  @Logged
  public double getMouthVelocityInRPM() {
    return mouthVelocity.getValue().in(RPM);
  }

  @Override
  public void periodic() {
    /* refresh all status signals */
    BaseStatusSignal.refreshAll(mouthVelocity);
  }
}
