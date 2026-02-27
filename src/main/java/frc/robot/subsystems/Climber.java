package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  private final CANBus canivore = new CANBus("drive");

  private final TalonFX leader = new TalonFX(0, canivore);
  private final TalonFX motor2 = new TalonFX(0, canivore);

  public Climber() {  
    
  }
}
