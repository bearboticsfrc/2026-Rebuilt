// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import bearlib.util.TunableNumber;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Flywheel extends SubsystemBase {
  /** Creates a new Flywheel. */
  // Create a new CANBus with name canivore
  private final CANBus canivore = new CANBus("drive");

  // Create the leader and follower TalonFX motors
  private final TalonFX leader = new TalonFX(60, canivore);

  // private final TalonFX follower = new TalonFX(22, canivore);

  // Velocity output control for the flywheel
  private final MotionMagicVelocityVoltage velocityOut = new MotionMagicVelocityVoltage(0);

  private final DutyCycleOut output = new DutyCycleOut(0);

  private TunableNumber rpm = new TunableNumber("RPM", 3600, () -> this.getTuningMode());

  private boolean getTuningMode() {
    return true;
  }

  // Tolerance for the flywheel velocity
  private final AngularVelocity tolerance = RotationsPerSecond.of(0.25);

  private final VoltageOut m_voltReq = new VoltageOut(0.0);

  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with Phoenix SignalLogger class
              (state) -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              (volts) -> leader.setControl(m_voltReq.withOutput(volts.in(Volts))), null, this));

  public Flywheel() {
    // Set the follower to follow the leader motor
    // follower.setControl(new Follower(leader.getDeviceID(), true));
    // Create and apply the configuration for the leader motor
    TalonFXConfiguration config = new TalonFXConfiguration();
    // Put's the motor in Coast mode to make it easier to move by hand
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // Configure the motor to make sure positive voltage is counter clockwise
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.Slot0.kS = 0.31576; // Static gain
    config.Slot0.kV = 0.11631; // Velocity gain
    config.Slot0.kA = 0.015927;
    config.Slot0.kP = .094744; // Proportional gain
    config.MotionMagic.MotionMagicCruiseVelocity = 9000; // Max velocity
    config.MotionMagic.MotionMagicAcceleration = 9000; // Max acceleration allowed
    // Try to apply config multiple time. Break after successfully applying
    for (int i = 0; i < 2; ++i) {
      var status = leader.getConfigurator().apply(config);
      if (status.isOK()) break;
    }
  }

  @Logged(name = "Target Angular Velocity")
  private AngularVelocity angularVelocity = RPM.of(rpm.get());

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println("value = " + rpm.get());
    if (rpm.hasChanged()) {
      angularVelocity = RPM.of(rpm.get());
      System.out.println("updating velocity to " + angularVelocity);
    }
  }

  /**
   * Sets the velocity for the flywheel.
   *
   * @param velocity The velocity to set.
   */
  public void setVelocity(AngularVelocity velocity) {
    // Apply the velocity output to the leader motor
    leader.setControl(velocityOut.withVelocity(velocity));
  }

  /**
   * Sets the percent for the flywheel.
   *
   * @param percent The percent to set.
   */
  public void setPercent(double value) {
    // Apply the velocity output to the leader motor
    leader.setControl(output.withOutput(.1));
  }

  /**
   * Command to run the flywheel at a slow speed.
   *
   * @return The command to run the flywheel slowly.
   */
  public Command runSlow() {
    // Command to run the flywheel at a slow speed
    return runOnce(() -> setVelocity(RPM.of(rpm.get())));
  }

  /**
   * Command to run the flywheel at a fast speed.
   *
   * @return The command to run the flywheel fast.
   */
  public Command runFast() {
    // Command to run the flywheel at a fast speed
    return runOnce(() -> setVelocity(DegreesPerSecond.of(1500)));
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
    return getVelocity()
        .isNear(
            getTargetVelocity(),
            tolerance); // Check if the current velocity is near the target velocity
  }

  /**
   * Gets the current velocity of the flywheel.
   *
   * @return The current velocity of the flywheel.
   */
  @Logged
  public AngularVelocity getVelocity() {
    // Get the current velocity of the flywheel
    return leader.getVelocity().getValue();
  }

  /**
   * Gets the target velocity for the flywheel.
   *
   * @return The target velocity of the flywheel.
   */
  @Logged
  public AngularVelocity getTargetVelocity() {
    // Return the target velocity
    return velocityOut.getVelocityMeasure();
  }

  @Logged
  public double getMotorCurrent() {
    return leader.getSupplyCurrent().refresh().getValueAsDouble();
  }

  @Logged
  public double getMotorTemp() {
    return leader.getDeviceTemp().refresh().getValueAsDouble();
  }

  // Stop the flywheel motors
  public void stop() {
    leader.stopMotor();
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
    return leader.getVelocity().getValue().in(RPM);
  }

  @Logged
  public double getTargetVelocityInRPM() {
    return velocityOut.getVelocityMeasure().in(RPM);
  }
}
