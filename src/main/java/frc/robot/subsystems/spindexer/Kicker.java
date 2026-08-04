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

public class Kicker extends Mechanism implements SelfTestable {

  private static final double kGearRatio = 2.5;

  private final VelocityVoltage velocityReq = new VelocityVoltage(0.0).withEnableFOC(true);

  private final AngularVelocity NORMAL_SPEED = RPM.of(2200);
  private final AngularVelocity SLOW_SPEED = RPM.of(200);

  private final AngularVelocity REVERSE_SPEED = RPM.of(-200);
  private final AngularVelocity REVERSE_SPEED_SLOW = RPM.of(-20);

  private static final AngularVelocity SELF_TEST_VELOCITY_THRESHOLD_RPM = RPM.of(200);

  @Logged private boolean selfTestPassed = false;

  private DCMotorSim simModel;

  public Kicker() {
    super("Kicker", CAN.KICKER, new CANBus(CAN.NAME));

    neutralMode(NeutralModeValue.Coast);
    inverted(InvertedValue.Clockwise_Positive);
    statorCurrentLimit(60);
    supplyCurrentLimit(60);
    kS(0.25);
    kV(0.32);
    kA(0.0);
    kP(0.2);
    sensorToMechanismRatio(kGearRatio);

    addConfig();

    if (Robot.isSimulation()) {
      simModel =
          simulationInitKrakenX60(motor, kGearRatio, 0.01, ChassisReference.Clockwise_Positive);
    }

    optimizeCAN();
    System.out.println(getName() + " Subsystem Initialized");
  }

  /** Simulation periodic. */
  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic(motor, kGearRatio, simModel);
  }

  /** Runs the kicker. */
  public Command run() {
    return run(() -> motor.setControl(velocityReq.withVelocity(NORMAL_SPEED)))
        .withName(getName() + ".Run");
  }

  /** Runs the kicker slow. */
  public Command runSlow() {
    return runOnce(() -> motor.setControl(velocityReq.withVelocity(SLOW_SPEED)))
        .withName(getName() + ".RunSlow");
  }

  /** Runs the kicker reverse. */
  public Command reverse() {
    return run(() -> motor.setControl(velocityReq.withVelocity(REVERSE_SPEED)))
        .withName(getName() + ".Reverse");
  }

  /** Runs the kicker reverse slowly. */
  public Command reverseSlow() {
    return runOnce(() -> motor.setControl(velocityReq.withVelocity(REVERSE_SPEED_SLOW)))
        .withName(getName() + ".Reverse");
  }

  /** Stops the kicker. */
  public Command stop() {
    return runOnce(() -> stopMotor()).withName(getName() + ".Stop");
  }

  /** Runnable to stop the kicker. */
  public void stopMotor() {
    motor.stopMotor();
  }

  /**
   * Signals whether the kicker is near a specific setpoint velocity.
   *
   * @param target The setpoint velocity.
   */
  private boolean isNearTarget(AngularVelocity target) {
    return motorVelocity.getValue().isNear(target, SELF_TEST_VELOCITY_THRESHOLD_RPM);
  }

  /**
   * Self tests at a setpoint velocity.
   *
   * @param target The setpoint velocity.
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
        .withName(getName() + ".SelfTestAt" + (int) target.in(RPM) + "RPM")
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
    return selfTestAt(SLOW_SPEED, "Robot/Tests/kicker/slow").withName(getName() + ".SelfTestSlow");
  }

  /** Self tests at normal speed. */
  @Override
  public Command selfTestFast() {
    return selfTestAt(NORMAL_SPEED, "Robot/Tests/kicker/fast")
        .withName(getName() + ".SelfTestFast");
  }

  /** The velocity in RPM. */
  @Logged(name = "velocityRPM")
  public double getVelocityInRPM() {
    return motorVelocity.getValue().in(RPM);
  }

  /** The setpoint velocity in RPM. */
  @Logged(name = "setpointRPM")
  public double getSetpointInRPM() {
    return velocityReq.Velocity * 60.0;
  }

  /** Signals whether or not the kicker is stopped. */
  @Logged
  public boolean isStopped() {
    return Math.abs(getVelocityInRPM()) <= 0.0;
  }
}
