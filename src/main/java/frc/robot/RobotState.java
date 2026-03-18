package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.field.AllianceFlipUtil;
import frc.robot.field.Field;
import java.util.function.Supplier;
import lombok.*;

public class RobotState {

  private static RobotState instance = new RobotState();

  @Logged public Translation2d turretTranslation;

  @Logged public Pose2d turretPose;

  public static final Transform2d turretToRobot =
      new Transform2d(Inches.of(-3.25), Inches.of(-3.25), new Rotation2d(Degrees.of(180)));

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  public boolean updatePoseInAutonomous = true;

  public boolean isRedAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red;
  }

  public boolean isBlueAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue;
  }

  @Getter @Setter public Pose2d robotPose = new Pose2d();
  @Getter @Setter public ChassisSpeeds robotVelocity = new ChassisSpeeds();
  @Getter @Setter public boolean shooting;

  @Logged
  public Rotation2d getRotation() {
    return robotPose.getRotation();
  }

  public void updatePose() {
    turretPose = robotPose.transformBy(turretToRobot);
    turretTranslation = (robotPose.transformBy(turretToRobot).getTranslation());
  }

  @Logged
  public ChassisSpeeds getFieldVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(robotVelocity, getRotation());
  }

  @Logged
  public boolean isInAllianceZone() {
    if (isBlueAlliance() && robotPose.getX() < Field.getMyAllianceLine().getX()) return true;
    else if (isRedAlliance() && robotPose.getX() > Field.getMyAllianceLine().getX()) return true;
    else return false;
  }

  @Logged
  public boolean isInNeutralZone() {
    if (isBlueAlliance() && robotPose.getX() > Field.getMyAllianceLine().getX()) return true;
    else if (isRedAlliance() && robotPose.getX() < Field.getMyAllianceLine().getX()) return true;
    else return false;
  }

  @Logged
  public boolean iSLeft() {
    return (isBlueAlliance() && robotPose.getY() > Field.getMyAllianceLine().getY()
        || isRedAlliance() && robotPose.getY() < Field.getMyAllianceLine().getY());
  }

  @Logged
  public boolean isLeftNeutralZone() {
    return (iSLeft() && isInNeutralZone());
  }

  @Logged
  public boolean isRightNeutralZone() {
    return (!iSLeft() && isInNeutralZone());
  }

  @Logged
  public boolean inDeadZone() {
    return (isInNeutralZone() && robotPose.getY() > 3.5 && robotPose.getY() < 4.7);
  }

  public Rotation2d getAngleToHub() {
    return AllianceFlipUtil.apply(
        Field.getMyHub().minus((robotPose.transformBy(turretToRobot).getTranslation())).getAngle());
  }

  public double getDistanceToHub() {
    return Field.getMyHub().getDistance((robotPose.transformBy(turretToRobot).getTranslation()));
  }

  public Supplier<Double> getLinearVelocity() {
    // replace with actual ratio
    return (isShooting()) ? () -> (getDistanceToHub() / 100) * 4.58 : () -> 4.58;
  }
}
