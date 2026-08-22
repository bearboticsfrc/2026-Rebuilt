package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;

import bearlib.Mechanism;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.rebuilt.CAN;
import frc.robot.test.SelfTestable;
import java.util.function.DoubleSupplier;

public class Flywheel extends Mechanism implements SelfTestable {

  private final MotionMagicVelocityVoltage velocityOut = new MotionMagicVelocityVoltage(0);

  private final double tolerance = 750;

  @Logged private boolean selfTestPassed = false;

  private DCMotorSim motorSimModel;

  private static final double SIM_GEAR_RATIO = 1.0;
  private static final double inertia = 0.004;

  public Flywheel() {
    super("FLYWHEEL", CAN.FLYWHEEL, new CANBus(CAN.NAME));

    neutralMode(NeutralModeValue.Coast);
    inverted(InvertedValue.CounterClockwise_Positive);
    kS(0.31576);
    kV(0.11941);
    kA(0.015595);
    kP(0.185);
    motionMagicCruiseVelocity(9000);
    motionMagicAcceleration(9000);
    peakForwardTorqueCurrent(100);
    peakReverseTorqueCurrent(0);
    peakForwardDutyCycle(1);
    peakReverseDutyCycle(0);
    statorCurrentLimit(Amps.of(80).in(Amps));
    supplyCurrentLimit(Amps.of(60).in(Amps));

    addConfig();

    if (Robot.isSimulation()) {
      motorSimModel =
          simulationInitKrakenX60(
              motor, SIM_GEAR_RATIO, inertia, ChassisReference.CounterClockwise_Positive);
    }

    System.out.println(getName() + " Subsystem Initialized");
    optimizeCAN();
  }

  /** Simulation periodic. */
  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic(motor, SIM_GEAR_RATIO, motorSimModel);
  }

  /**
   * Runnable to set flywheel velocity.
   *
   * @param velocity The velocity setpoint.
   */
  public void setVelocity(AngularVelocity velocity) {
    motor.setControl(velocityOut.withVelocity(velocity));
  }

  /**
   * Runs the flywheel at a specific velocity in RPM.
   *
   * @param rpm The target velocity in RPM.
   */
  public Command runAtSpeed(double rpm) {
    return runOnce(() -> setVelocity(RPM.of(rpm))).withName(getName() + ".runAtSpeed(double)");
  }

  /**
   * Runs flywheel at a specific velocity in RPM
   *
   * @param rpm The target velocity in RPM
   */
  public Command runAtSpeed(DoubleSupplier rpm) {
    if (!RobotState.getInstance().isInNeutralZone().getAsBoolean()) {
      RobotState.getInstance().setShooting(true);
    }
    return run(() -> setVelocity(RPM.of(rpm.getAsDouble())))
        .withName(getName() + ".runAtSpeed(supplier)");
  }

  /** Stops the flywheel. */
  public Command stopCommand() {
    RobotState.getInstance().setShooting(false);
    return runOnce(() -> stop());
  }

  /** Signals whether or not the flywheel is at its setpoint. */
  @Logged
  public boolean isAtTarget() {
    return getTargetVelocityInRPM() > 0
        && Math.abs(getVelocityInRPM() - getTargetVelocityInRPM()) < tolerance;
  }

  /** Signals whether or not the flywheel is stopped. */
  @Logged
  public boolean isStopped() {
    return Math.abs(getVelocityInRPM()) <= 0.5;
  }

  /** Runnable to stop flywheel. */
  public void stop() {
    motor.stopMotor();
  }

  /**
   * Self tests at a specifc target velocity.
   *
   * @param target The target velocity.
   * @param ntKey The NT key
   */
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

  /** Self tests at slow speed. */
  @Override
  public Command selfTestSlow() {
    return selfTestAt(RPM.of(1000), "Robot/Tests/flywheel/slow")
        .withName(getName() + ".SelfTestSlow");
  }

  /** Self tests at normal speed. */
  @Override
  public Command selfTestFast() {
    return selfTestAt(RPM.of(3150), "Robot/Tests/flywheel/fast")
        .withName(getName() + ".SelfTestFast");
  }

  /** The velocity in RPM. */
  @Logged(name = "velocityRPM")
  public double getVelocityInRPM() {
    return motorVelocity.getValue().in(RPM);
  }

  /** The target velocity in RPM. */
  @Logged(name = "targetVelocityRPM")
  public double getTargetVelocityInRPM() {
    return velocityOut.getVelocityMeasure().in(RPM);
  }
}
