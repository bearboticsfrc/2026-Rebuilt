// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;

public class Flywheel extends SubsystemBase {
  /** Creates a new Flywheel. */
  // Create a new CANBus with name canivore
  private final CANBus canivore = new CANBus("Default Name");

  private final TalonFX motor = new TalonFX(26, canivore);

  // Velocity output control for the flywheel

  // private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC = new
  // VelocityTorqueCurrentFOC(0);
  // private final VelocityDutyCycle velocityDutyCycle = new VelocityDutyCycle(0);

  private final MotionMagicVelocityVoltage velocityOut = new MotionMagicVelocityVoltage(0);
  // private final DutyCycleOut output = new DutyCycleOut(0);

  // Tolerance for the flywheel velocity
  private final double tolerance = 750; // RPM

  private final VoltageOut m_voltReq = new VoltageOut(0.0);

  private final StatusSignal<Current> motorCurrent = motor.getSupplyCurrent(false);
  private final StatusSignal<AngularVelocity> motorVelocity = motor.getVelocity(false);

  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with Phoenix SignalLogger class
              (state) -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              (volts) -> motor.setControl(m_voltReq.withOutput(volts.in(Volts))), null, this));

  public Flywheel() {
    super("Flywheel");

    TalonFXConfiguration config = new TalonFXConfiguration();
    // Put's the motor in Coast mode to make it easier to move by hand
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    // Adjusted the values for VelocityTorqueCurrentFOC which uses amps instead of volts
    config.Slot0.kS = 0.31576; // Static gain
    config.Slot0.kV = 0.11941; // Velocity gain
    config.Slot0.kA = 0.015595;
    config.Slot0.kP = .185; // Proportional gain
    config.MotionMagic.MotionMagicCruiseVelocity = 9000; // Max velocity
    config.MotionMagic.MotionMagicAcceleration = 9000; // Max acceleration allowed
    config.TorqueCurrent.PeakForwardTorqueCurrent = 100;
    config.TorqueCurrent.PeakReverseTorqueCurrent = 0;
    config.MotorOutput.PeakForwardDutyCycle = 1;
    config.MotorOutput.PeakReverseDutyCycle = 0;
    // Try to apply config multiple time. Break after successfully applying
    StatusCode status = motor.getConfigurator().apply(config);

    for (int i = 0; i < 2; ++i) {
      if (status.isOK()) break;

      status = motor.getConfigurator().apply(config);
    }

    if (!status.isOK()) {
      System.out.println("ERROR Configuring Flywheel motor: " + status);
    }

    System.out.println("Flywheel Subsystem Initialized");

    optimizeCAN();
  }

  private void optimizeCAN() {
    motor.getPosition().setUpdateFrequency(50);
    motor.getVelocity().setUpdateFrequency(50);
    motor.getSupplyCurrent().setUpdateFrequency(50);
    motor.getDeviceTemp().setUpdateFrequency(10);

    motor.optimizeBusUtilization();
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(motorCurrent, motorVelocity);
  }

  public void setVelocity(AngularVelocity velocity) {
    // motor.setControl(new VelocityDutyCycle(velocity));
    motor.setControl(velocityOut.withVelocity(velocity));
  }

  /**
   * Command to run the flywheel at a given speed.
   *
   * @return The command to run the flywheel at the given speed.
   */
  public Command runAtSpeed(double rpm) {
    // Command to run the flywheel at a given speed
    return runOnce(() -> setVelocity(RPM.of(rpm)));
  }

  /**
   * Command to run the flywheel at a given speed.
   *
   * @return The command to run the flywheel at the given speed.
   */
  public Command runAtSpeed(DoubleSupplier rpm) {
    // Command to run the flywheel at a given speed
    return runOnce(() -> setVelocity(RPM.of(rpm.getAsDouble())));
  }

  /**
   * Command to stop the flywheel.
   *
   * @return The command to stop the flywheel.
   */
  public Command stopCommand() {
    return runOnce(() -> stop());
  }

  /**
   * Checks if the flywheel is at its target speed.
   *
   * @return true if at target speed, false otherwise
   */
  @Logged
  public boolean isAtTarget() {
    return Math.abs(getVelocityInRPM() - getTargetVelocityInRPM())
        < tolerance; // Check if the current velocity is near the target velocity
  }

  @Logged
  public double getMotorCurrent() {
    return motorCurrent.getValueAsDouble();
  }

  // Stop the flywheel motors
  public void stop() {
    motor.stopMotor();
  }

  /**
   * Returns a command that will execute a quasistatic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  @Logged
  public double getVelocityInRPM() {
    return motorVelocity.getValue().in(RPM);
  }

  @Logged
  public double getTargetVelocityInRPM() {
    return velocityOut.getVelocityMeasure().in(RPM);
  }
}
