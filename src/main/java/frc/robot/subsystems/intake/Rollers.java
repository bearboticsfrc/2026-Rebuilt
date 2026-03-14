package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rollers extends SubsystemBase {

  private final CANBus rio = new CANBus("rio");

  private final TalonFX motor = new TalonFX(25, rio);

  private final DutyCycleOut dutyReq = new DutyCycleOut(0.0);

  public final double ROLLER_SPEED = 0.65;

  public final double ROLLER_SPEED_SLOW = 0.3;

  public Rollers() {
    super("Intake");
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    StatusCode status = motor.getConfigurator().apply(config);

    for (int i = 0; i < 2; ++i) {
      if (status.isOK()) break;
      status = motor.getConfigurator().apply(config);
    }
    if (!status.isOK()) {
      System.out.println("ERROR Configuring Intake roller motor: " + status);
    }

    System.out.println("Intake Subsystem Initialized");
  }

  public void setOutput(double output) {
    motor.setControl(dutyReq.withOutput(output));
  }

  public Command run() {
    return runOnce(() -> setOutput(ROLLER_SPEED)).withName("RollersRun");
  }

  public Command runSlow() {
    return runOnce(() -> setOutput(ROLLER_SPEED_SLOW));
  }

  public Command stop() {
    return runOnce(() -> motor.stopMotor()).withName("RollersStop");
  }

  @Logged
  public double getRollerVelocityInRPM() {
    return motor.getVelocity().getValue().in(RPM);
  }
}
