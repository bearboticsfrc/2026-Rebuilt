package frc.robot.subsystems.spindexer;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.util.PhoenixUtil.applyConfig;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.CAN;
import frc.robot.Copilot;
import frc.robot.Mechanism;
import frc.robot.Robot;
import frc.robot.test.SelfTestable;
import java.util.Set;

public class Spindexer extends Mechanism implements SelfTestable {

  private static final double kGearRatio = 7.2;

  private final CANBus canivore = new CANBus(CAN.NAME);
  private final TalonFX motor = new TalonFX(CAN.SPINDEXER, canivore);

  private final VelocityVoltage velocityReq =
      new VelocityVoltage(0.0).withEnableFOC(true).withSlot(0);
  private final MotionMagicVoltage positionReq =
      new MotionMagicVoltage(0.0).withEnableFOC(true).withSlot(1);

  private final AngularVelocity NORMAL_SPEED = RPM.of(600);
  private final AngularVelocity SLOW_SPEED = RPM.of(60);
  private final AngularVelocity REVERSE_SPEED = RPM.of(-200);
  private final AngularVelocity REVERSE_SPEED_SLOW = RPM.of(-60);

  private final StatusSignal<Angle> position = motor.getPosition(false);

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

    applyConfig(() -> motor.getConfigurator().apply(config), getName());

    if (Robot.isSimulation()) {
      simModel =
          simulationInitKrakenX60(
              motor, kGearRatio, 0.01, ChassisReference.CounterClockwise_Positive);
    }

    optimizeCAN();
    buttonMappings();
    System.out.println(getName() + " Subsystem Initialized");
  }

  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic(motor, kGearRatio, simModel);
  }

  /* Commands */

  public Command run() {
    return runOnce(() -> motor.setControl(velocityReq.withVelocity(NORMAL_SPEED)))
        .withName(getName() + ".Run");
  }

  public Command runSlow() {
    return runOnce(() -> motor.setControl(velocityReq.withVelocity(SLOW_SPEED)))
        .withName(getName() + ".RunSlow");
  }

  public Command reverse() {
    return run(() -> motor.setControl(velocityReq.withVelocity(REVERSE_SPEED)))
        .withName(getName() + ".Reverse");
  }

  public Command reverseSlow() {
    return runOnce(() -> motor.setControl(velocityReq.withVelocity(REVERSE_SPEED_SLOW)))
        .withName(getName() + ".ReverseSlow");
  }

  public Command stop() {
    return runOnce(() -> stopMotor()).withName(getName() + ".Stop");
  }

  public void stopMotor() {
    motor.stopMotor();
  }

  public Command oscillate() {
    return Commands.defer(
            () -> {
              Angle startPosition = position.getValue();
              Angle forwardTarget = startPosition.plus(Rotations.of(.25));
              Angle reverseTarget = startPosition;

              return run(() ->
                      motor.setControl(positionReq.withPosition(forwardTarget).withEnableFOC(true)))
                  .until(() -> position.getValue().isNear(forwardTarget, Rotations.of(.05)))
                  .andThen(
                      run(() ->
                              motor.setControl(
                                  positionReq.withPosition(reverseTarget).withEnableFOC(true)))
                          .until(
                              () -> position.getValue().isNear(reverseTarget, Rotations.of(.05))))
                  .repeatedly();
            },
            Set.of(this))
        .withName(getName() + ".oscillate");
  }

  /* Self Test */

  private boolean isNearTarget(AngularVelocity target) {
    return motorVelocity.getValue().isNear(target, SELF_TEST_VELOCITY_THRESHOLD_RPM);
  }

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

  @Override
  public Command selfTestSlow() {
    return selfTestAt(SLOW_SPEED, "Robot/Tests/spindexer/slow")
        .withName(getName() + ".SelfTestSlow");
  }

  @Override
  public Command selfTestFast() {
    return selfTestAt(NORMAL_SPEED, "Robot/Tests/spindexer/fast")
        .withName(getName() + ".SelfTestFast");
  }

  @Logged(name = "velocityRPM")
  public double getVelocityInRPM() {
    return motorVelocity.getValue().in(RPM);
  }

  @Logged(name = "setpointRPM")
  public double getSetpointInRPM() {
    return velocityReq.Velocity * 60.0;
  }

  @Logged
  public boolean isStopped() {
    return Math.abs(getVelocityInRPM()) <= 0.0;
  }

  /* Button Mappings for Copilot */

  public void buttonMappings() {
    Copilot.spindexerIdle().onTrue(stop());
    Copilot.spindexerFwdSlow().onTrue(runSlow()).onFalse(stop());
    Copilot.spindexerFwdFast().onTrue(run()).onFalse(stop());
    Copilot.spindexerRevSlow().onTrue(reverseSlow()).onFalse(stop());
    Copilot.spindexerRevFast().onTrue(reverse()).onFalse(stop());
  }
}
