package frc.robot.subsystems.Turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.epilogue.Logged;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;

public class TurretAbsolutePosition {
  CANBus canbus = new CANBus("drive");

  private final CANcoder turretCANcoderMain = new CANcoder(31, canbus); // 10 tooth
  private final CANcoder turretCANcoderAux = new CANcoder(32, canbus); // 11 tooth

  private EasyCRT absoluteEncoder = new EasyCRT(getEasyCRTConfig());

  private EasyCRTConfig getEasyCRTConfig() {
    return new EasyCRTConfig(
            turretCANcoderMain.getAbsolutePosition().asSupplier(),
            turretCANcoderAux.getAbsolutePosition().asSupplier())
        .withCommonDriveGear(1, 93, 10, 11)
        .withCrtGearRecommendationInputs(93, 1.0)
        .withMechanismRange(Rotations.of(0.0), Rotations.of(1.0))
        .withMatchTolerance(Degrees.of(20))
        .withAbsoluteEncoder1Inverted(true)
        .withAbsoluteEncoder2Inverted(false)
        .withAbsoluteEncoderOffsets(Rotations.of(-.343018), Rotations.of(-.451904));
  }

  // @Logged
  public double getAbsoluteAngle() {
    return absoluteEncoder.getAngleOptional().orElse(Degrees.of(-1)).in(Degrees);
  }

  // @Logged
  public String getAbsoluteAngleStatus() {
    return absoluteEncoder.getLastStatus();
  }

  // @Logged
  public double getAbsoluteAngleIterations() {
    return absoluteEncoder.getLastIterations();
  }

  @Logged
  public double getAbsoluteAngleMain() {
    return turretCANcoderMain.getAbsolutePosition().getValue().in(Degrees);
  }

  @Logged
  public double getAbsoluteAngleAux() {
    return turretCANcoderAux.getAbsolutePosition().getValue().in(Degrees);
  }
}
