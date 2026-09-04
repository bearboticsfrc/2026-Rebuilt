package frc.robot.subsystems.spindexer;

import static edu.wpi.first.units.Units.RPM;

import bearlib.Mechanism;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.VelocityVoltage;
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
import frc.robot.rebuilt.CAN;
import frc.robot.test.SelfTestable;

public class Spindexer extends Mechanism implements SelfTestable {

  private static final double kGearRatio = 7.2;

  private final VelocityVoltage velocityReq =
      new VelocityVoltage(0.0).withEnableFOC(true).withSlot(0);

  private final AngularVelocity NORMAL_SPEED = RPM.of(600);
  private final AngularVelocity SLOW_SPEED = RPM.of(60);
  private final AngularVelocity REVERSE_SPEED = RPM.of(-200);
  private final AngularVelocity REVERSE_SPEED_SLOW = RPM.of(-60);

  private DCMotorSim simModel;

  @Logged private boolean selfTestPassed = false;

  private static final AngularVelocity SELF_TEST_VELOCITY_THRESHOLD_RPM = RPM.of(25);

  public Spindexer() {
    super("Spindexer", CAN.SPINDEXER, new CANBus(CAN.NAME));

    statorCurrentLimit(60);
    supplyCurrentLimit(60);
    neutralMode(NeutralModeValue.Coast);
    inverted(InvertedValue.CounterClockwise_Positive);
    kS(0.2);
    kV(0.75);
    kA(0);
    kP(0.2);

    addConfig();

    if (Robot.isSimulation()) {
      simModel =
          simulationInitKrakenX60(
              motor, kGearRatio, 0.01, ChassisReference.CounterClockwise_Positive);
    }

    optimizeCAN();
    System.out.println(getName() + " Subsystem Initialized");
  }

  /** Simulation Periodic. */
  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic(motor, kGearRatio, simModel);
  }

  /** Runs the spindexer at normal speed. */
  public Command run() {
    return runOnce(() -> motor.setControl(velocityReq.withVelocity(NORMAL_SPEED)))
        .withName(getName() + ".Run");
  }

  /** Runs the spindexer at a slow speed. */
  public Command runSlow() {
    return runOnce(() -> motor.setControl(velocityReq.withVelocity(SLOW_SPEED)))
        .withName(getName() + ".RunSlow");
  }

  /** Runs the spindexer reverse at normal speed. */
  public Command reverse() {
    return run(() -> motor.setControl(velocityReq.withVelocity(REVERSE_SPEED)))
        .withName(getName() + ".Reverse");
  }

  /** Runs the spindexer reverse at slow speed. */
  public Command reverseSlow() {
    return runOnce(() -> motor.setControl(velocityReq.withVelocity(REVERSE_SPEED_SLOW)))
        .withName(getName() + ".ReverseSlow");
  }

  /** Stops the spindexer. */
  public Command stop() {
    return runOnce(() -> stopMotor()).withName(getName() + ".Stop");
  }

  /** Runnable for stop(). */
  public void stopMotor() {
    motor.stopMotor();
  }

  /**
   * Signals if the spindexer is near a target velocity.
   *
   * @param target The target velocity.
   */
  private boolean isNearTarget(AngularVelocity target) {
    return motorVelocity.getValue().isNear(target, SELF_TEST_VELOCITY_THRESHOLD_RPM);
  }

  /**
   * Self tests the motor at a specific velocity.
   *
   * @param target The specific velocity.
   * @param ntKey The NT key.
   */
  private Command selfTestAt(AngularVelocity target, String ntKey) {
    return Commands.runOnce(
            () -> {
              var nt = NetworkTableInstance.getDefault();
              nt.getEntry(ntKey + "/message").setString("Running...");
              nt.getEntry(ntKey + "/passed").unpublish();
              ;
            })
        .andThen(runOnce(() -> motor.setControl(velocityReq.withVelocity(target))))
        .withName("Spindexer.SelfTestAt" + (int) target.in(RPM) + "RPM")
        .andThen(Commands.waitSeconds(4))
        .andThen(Commands.waitUntil(() -> isNearTarget(target)).withTimeout(2.0))
        .andThen(
            runOnce(
                () -> {
                  selfTestPassed = isNearTarget(target);
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
    return selfTestAt(SLOW_SPEED, "Robot/Tests/spindexer/slow")
        .withName(getName() + ".SelfTestSlow");
  }

  /** Self tests at fast speed. */
  @Override
  public Command selfTestFast() {
    return selfTestAt(NORMAL_SPEED, "Robot/Tests/spindexer/fast")
        .withName(getName() + ".SelfTestFast");
  }

  /** The spindexer velocity in rpm. */
  @Logged(name = "velocityRPM")
  public double getVelocityInRPM() {
    return motorVelocity.getValue().in(RPM);
  }

  /** The spindexer setpoint velocity in rpm. */
  @Logged(name = "setpointRPM")
  public double getSetpointInRPM() {
    return velocityReq.Velocity * 60.0;
  }

  /** Signals whether the spindexer is stopped. */
  @Logged
  public boolean isStopped() {
    return Math.abs(getVelocityInRPM()) <= 0.0;
  }
}
