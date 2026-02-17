package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private final TalonFX rotation;
  private final double rotatedOut = 0.0; // Replace with actual retracted position
  private final double rotatedIn = 1.0; // Replace with actual extended position
  private final PositionVoltage m_positionRequest = new PositionVoltage(0);
  private final TalonFX extension;
  private final double extendedOut = 0.0; // Replace with actual retracted position
  private final double extendedIn = 1.0; // Replace with actual extended position

  public Climber() {
    extension = new TalonFX(1); // Replace 1 with actual CAN ID for extendoPatronum
    rotation = new TalonFX(2); // Replace 2 with actual CAN ID for rotato
  }

  // Rotates climber out/in
  // extends the climber
  public Command extendArm() {
    double currentPosition = rotation.getPosition().getValueAsDouble();

    return (currentPosition < rotatedOut)
        ? Commands.run(() -> rotation.setControl(m_positionRequest.withPosition(rotatedOut)))
        : Commands.run(() -> rotation.setControl(m_positionRequest.withPosition(currentPosition)));
  }

  // retracts the climber
  public Command retractArm() {
    double currentPosition = extension.getPosition().getValueAsDouble();

    return (currentPosition > rotatedIn)
        ? Commands.run(() -> extension.setControl(m_positionRequest.withPosition(rotatedIn)))
        : Commands.run(
            () -> extension.setControl(m_positionRequest.withPosition(currentPosition)));
  }

  // stops the climber
  public void stop() {
    extension.set(0.0);
  }

  // retract / extend
  // extends the climber
  public Command extendClimb() {
    double currentPosition = extension.getPosition().getValueAsDouble();

    return (currentPosition < extendedOut)
        ? Commands.run(
            () -> extension.setControl(m_positionRequest.withPosition(extendedOut)))
        : Commands.run(
            () -> extension.setControl(m_positionRequest.withPosition(currentPosition)));
  }

  // retracts the climber
  public Command retractClimb() {
    double currentPosition = extension.getPosition().getValueAsDouble();

    return (currentPosition > extendedIn)
        ? Commands.run(() -> extension.setControl(m_positionRequest.withPosition(extendedIn)))
        : Commands.run(
            () -> extension.setControl(m_positionRequest.withPosition(currentPosition)));
  }

  // Climb
  public Command climb() {
    return Commands.runOnce(
        () -> {
          extendArm().schedule();
          extendClimb().schedule();
          // add drive forward to tower x command
          retractClimb().schedule();
        });
  }

  // Descend
  public Command decend() {
    return Commands.runOnce(
        () -> {
          extendClimb().schedule();
          // add drive backward slightly command
          retractClimb().schedule();
          retractArm().schedule();
        });
  }
}
