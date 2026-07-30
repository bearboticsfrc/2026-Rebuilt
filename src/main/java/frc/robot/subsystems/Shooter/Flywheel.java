// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.PhoenixUtil.applyConfig;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.CAN;
import frc.robot.Copilot;
import frc.robot.Mechanism;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.test.SelfTestable;
import java.util.function.DoubleSupplier;

public class Flywheel extends Mechanism implements SelfTestable {

  // Velocity output control for the flywheel
  private final MotionMagicVelocityVoltage velocityOut = new MotionMagicVelocityVoltage(0);

  // Tolerance for the flywheel velocity
  private final double tolerance = 750; // RPM

  private final VoltageOut m_voltReq = new VoltageOut(0.0);

  @Logged private boolean selfTestPassed = false;

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

  private DCMotorSim motorSimModel;

  private static final double SIM_GEAR_RATIO = 1.0;
  private static final double inertia = 0.004;

  public Flywheel() {
    super("FLYWHEEL", CAN.FLYWHEEL, new CANBus(CAN.NAME));

    TalonFXConfiguration config = new TalonFXConfiguration();
    // Put's the motor in Coast mode to make it easier to move by hand
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

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
    config.CurrentLimits.StatorCurrentLimit = Amps.of(80).in(Amps);
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = Amps.of(60).in(Amps);
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Try to apply config multiple time. Break after successfully applying
    applyConfig(() -> motor.getConfigurator().apply(config), getName());

    if (Robot.isSimulation()) {
      motorSimModel =
          simulationInitKrakenX60(
              motor, SIM_GEAR_RATIO, inertia, ChassisReference.CounterClockwise_Positive);
    }

    System.out.println(getName() + " Subsystem Initialized");
    buttonMappings();
    optimizeCAN();
  }

  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic(motor, SIM_GEAR_RATIO, motorSimModel);
  }

  /* Commands */

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
    return runOnce(() -> setVelocity(RPM.of(rpm))).withName(getName() + ".runAtSpeed(double)");
  }

  public Command warmUp() {
    return runAtSpeed(() -> 1600.0);
  }

  /**
   * Command to run the flywheel at a given speed.
   *
   * @return The command to run the flywheel at the given speed.
   */
  public Command runAtSpeed(DoubleSupplier rpm) {
    // Command to run the flywheel at a given speed\
    if (!RobotState.getInstance().isInNeutralZone().getAsBoolean()) {
      RobotState.getInstance().setShooting(true);
    }
    return run(() -> setVelocity(RPM.of(rpm.getAsDouble())))
        .withName(getName() + ".runAtSpeed(supplier)");
  }

  /**
   * Command to stop the flywheel.
   *
   * @return The command to stop the flywheel.
   */
  public Command stopCommand() {
    RobotState.getInstance().setShooting(false);
    return runOnce(() -> stop());
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

  /* Self Test */

  /**
   * Checks if the flywheel is at its target speed.
   *
   * @return true if at target speed, false otherwise
   */
  @Logged
  public boolean isAtTarget() {
    return getTargetVelocityInRPM() > 0
        && Math.abs(getVelocityInRPM() - getTargetVelocityInRPM())
            < tolerance; // Check if the current velocity is near the target velocity
  }

  @Logged
  public boolean isStopped() {
    return Math.abs(getVelocityInRPM()) <= 0.5;
  }

  // Stop the flywheel motors
  public void stop() {
    motor.stopMotor();
  }

  private Command selfTestAt(AngularVelocity target, String ntKey) {
    return Commands.runOnce(
            () -> {
              var nt = NetworkTableInstance.getDefault();
              nt.getEntry(ntKey + "/message").setString("Running...");
              nt.getEntry(ntKey + "/passed").unpublish();
              ;
            })
        .andThen(runAtSpeed(() -> target.in(RPM)))
        .withTimeout(4)
        .andThen(Commands.waitUntil(this::isAtTarget).withTimeout(2.0))
        .andThen(
            runOnce(
                () -> {
                  selfTestPassed = isAtTarget();
                  String result =
                      (selfTestPassed ? "PASS" : "FAIL")
                          + ": "
                          + (int) getVelocityInRPM()
                          + " RPM (target "
                          + (int) target.in(RPM)
                          + " RPM)";
                  var nt = NetworkTableInstance.getDefault();
                  nt.getEntry(ntKey + "/passed").setBoolean(selfTestPassed);
                  nt.getEntry(ntKey + "/message").setString(result);
                }))
        .finallyDo(() -> motor.stopMotor());
  }

  @Override
  public Command selfTestSlow() {
    return selfTestAt(RPM.of(1000), "Robot/Tests/flywheel/slow")
        .withName(getName() + ".SelfTestSlow");
  }

  @Override
  public Command selfTestFast() {
    return selfTestAt(RPM.of(3150), "Robot/Tests/flywheel/fast")
        .withName(getName() + ".SelfTestFast");
  }

  /* Logged Values */

  @Logged(name = "velocityRPM")
  public double getVelocityInRPM() {
    return motorVelocity.getValue().in(RPM);
  }

  @Logged(name = "targetVelocityRPM")
  public double getTargetVelocityInRPM() {
    return velocityOut.getVelocityMeasure().in(RPM);
  }

  /* Simulation */

  public void buttonMappings() {
    Copilot.flywheelIdle().onTrue(stopCommand());
    Copilot.flywheel500().onTrue(runAtSpeed(500.0));
    Copilot.flywheel1200().onTrue(runAtSpeed(1200.0));
    Copilot.flywheel3700().onTrue(runAtSpeed(3700.0));
  }
}
