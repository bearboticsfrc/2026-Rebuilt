package frc.robot.subsystems;

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

public class Intake extends SubsystemBase {

  private final CANBus rio = new CANBus("rio");

  private final TalonFX roller = new TalonFX(25, rio);

  private final DutyCycleOut dutyReq = new DutyCycleOut(0.0);

  public final double ROLLER_SPEED = 0.5;

  public Intake() {
    super("Intake");
    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();

    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    StatusCode status = roller.getConfigurator().apply(rollerConfig);

    for (int i = 0; i < 2; ++i) {
      if (status.isOK()) break;
      status = roller.getConfigurator().apply(rollerConfig);
    }
    if (!status.isOK()) {
      System.out.println("ERROR Configuring Intake roller motor: " + status);
    }

    System.out.println("Intake Subsystem Initialized");
  }

  public void setRollerOutput(double output) {
    roller.setControl(dutyReq.withOutput(output));
  }

  public void stopRoller() {
    roller.stopMotor();
  }

  public Command runRoller() {
    return runOnce(() -> setRollerOutput(ROLLER_SPEED));
  }

  public Command stopRollerCommand() {
    return runOnce(() -> stopRoller());
  }

  @Logged
  public double getRollerVelocityInRPM() {
    return roller.getVelocity().getValue().in(RPM);
  }
}
