package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  private final CANBus canivore = new CANBus("Default Name");

  private final TalonFX leader = new TalonFX(19, canivore);
  private final TalonFX motor2 = new TalonFX(15, canivore);

  public Climber() {}
}
