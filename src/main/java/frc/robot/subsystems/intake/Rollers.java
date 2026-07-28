package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RPM;
import static frc.robot.util.PhoenixUtil.applyConfig;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.CAN;
import frc.robot.Copilot;
import frc.robot.Mechanism;
import frc.robot.Robot;
import frc.robot.test.SelfTestable;

public class Rollers extends Mechanism implements SelfTestable {

  private static final String NAME = "Rollers";

  private final VelocityTorqueCurrentFOC velocityTorqueCurrent = new VelocityTorqueCurrentFOC(0.0);

  // Theoretical max == 5400
  public final AngularVelocity ROLLER_SPEED = RPM.of(5000);
  public final AngularVelocity ROLLER_SPEED_REVERSE = RPM.of(-5000);

  public final AngularVelocity ROLLER_SPEED_SLOW = RPM.of(3500);
  public final AngularVelocity ROLLER_SPEED_SLOW_REVERSE = RPM.of(-3500);

  private final double gearRatio = 1.11;

  private static final AngularVelocity VELOCITY_TOLERANCE = RPM.of(200);

  @Logged private boolean selfTestPassed = false;

  private DCMotorSim motorSimModel;

  public Rollers() {

    super(NAME, CAN.ROLLERS, new CANBus(CAN.NAME));

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.CurrentLimits.StatorCurrentLimit = 80;
    config.CurrentLimits.SupplyCurrentLimit = 60;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    // TorqueCurrentFOC gains — units are amps, not volts
    config.Slot0.kS = 0.5; // amps to overcome static friction — tune on real robot
    config.Slot0.kV = 0.12; // amps per mechanism RPS — tune for steady-state accuracy under load
    config.Slot0.kP = 6.0; // amps per RPS of error — tune if residual error under load
    config.Slot0.kA = 0.0; // leave at 0, not needed for simple rollers

    config.Feedback.SensorToMechanismRatio = gearRatio;

    applyConfig(() -> motor.getConfigurator().apply(config), getName());

    optimizeCAN();

    if (Robot.isSimulation()) {
      motorSimModel =
          simulationInitKrakenX60(
              motor, gearRatio, 0.001, ChassisReference.CounterClockwise_Positive);
    }

    buttonMappings();
    System.out.println(getName() + " Subsystem Initialized");
  }

  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic(motor, gearRatio, motorSimModel);
  }

  /* Commands */

  private void setOutput(AngularVelocity velocity) {
    motor.setControl(velocityTorqueCurrent.withVelocity(velocity));
  }

  public Command run() {
    return runOnce(() -> setOutput(ROLLER_SPEED)).withName(NAME + ".Run");
  }

  public Command runReverse() {
    return runOnce(() -> setOutput(ROLLER_SPEED_REVERSE)).withName(NAME + ".RunReverse");
  }

  public Command runSlow() {
    return runOnce(() -> setOutput(ROLLER_SPEED_SLOW)).withName(NAME + ".RunSlow");
  }

  public Command runSlowReverse() {
    return runOnce(() -> setOutput(ROLLER_SPEED_SLOW_REVERSE)).withName(NAME + ".RunSlowReverse");
  }

  public Command stop() {
    return runOnce(() -> motor.stopMotor()).withName(NAME + ".Stop");
  }

  /* Self Test */

  private boolean isNearTarget(AngularVelocity target) {
    return motorVelocity.getValue().isNear(target, VELOCITY_TOLERANCE);
  }

  private Command selfTestAt(AngularVelocity target, String ntKey) {
    return Commands.runOnce(
            () -> {
              var nt = NetworkTableInstance.getDefault();
              nt.getEntry(ntKey + "/message").setString("Running...");
              nt.getEntry(ntKey + "/passed").unpublish();
              ;
            })
        .andThen(runOnce(() -> setOutput(target)))
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
    return selfTestAt(ROLLER_SPEED_SLOW, "Robot/Tests/rollers/slow")
        .withName(getName() + ".SelfTestSlow");
  }

  @Override
  public Command selfTestFast() {
    return selfTestAt(ROLLER_SPEED, "Robot/Tests/rollers/fast")
        .withName(getName() + ".SelfTestFast");
  }

  /* Logged Values */

  @Logged(name = "setpointRPM")
  public double getSetpoint() {
    return velocityTorqueCurrent.Velocity * 60.0;
  }

  @Logged(name = "velocityRPM")
  public double getVelocityInRPM() {
    return motorVelocity.getValue().in(RPM);
  }

  /* Button Mappings for Copilot */

  public void buttonMappings() {
    Copilot.rollerIdle().onTrue(stop());
    Copilot.rollerFwdSlow().onTrue(runSlow()).onFalse(stop());
    Copilot.rollerFwdFast().onTrue(run()).onFalse(stop());
    Copilot.rollerRevSlow().onTrue(runSlowReverse()).onFalse(stop());
    Copilot.rollerRevFast().onTrue(runReverse()).onFalse(stop());
  }
}
