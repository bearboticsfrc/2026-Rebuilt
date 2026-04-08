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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.field.Field;
import frc.robot.subsystems.DynamicShootingCalculator;
import lombok.*;

public class RobotState {

  private static RobotState instance = new RobotState();

  @Logged public Translation2d turretTranslation;

  @Logged @Getter public Pose2d turretPose;

  public static final Transform2d turretToRobot =
      new Transform2d(Inches.of(-6.25), Inches.of(-6.25), new Rotation2d());

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  @Logged public Trigger stopped = new Trigger(() -> isStopped());

  public boolean updatePoseInAutonomous = true;

  public boolean isRedAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red;
  }

  public boolean isBlueAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue;
  }

  @Logged @Getter @Setter public Pose2d robotPose = new Pose2d();
  @Getter @Setter public ChassisSpeeds robotVelocity = new ChassisSpeeds();
  @Logged @Getter @Setter public boolean shooting;

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

  // needs work
  public boolean isStopped() {
    if (isInAllianceZone()) {
      return getFieldVelocity().vxMetersPerSecond < 0.00005
          && getFieldVelocity().vyMetersPerSecond < 0.00005
          && Math.abs(getFieldVelocity().omegaRadiansPerSecond) < 0.00005;
    }
    return false;
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
    return Field.getMyHub()
        .minus((robotPose.transformBy(turretToRobot).getTranslation()))
        .getAngle()
        .plus(new Rotation2d(Degrees.of(180)));
  }

  public double getDistanceToHub() {
    return Field.getMyHub().getDistance((robotPose.transformBy(turretToRobot).getTranslation()));
  }

  @Logged
  public double getTargetDistance() {
    if (isInNeutralZone()) {
      return (iSLeft())
          ? Field.getMyLeft().getDistance((robotPose.transformBy(turretToRobot).getTranslation()))
          : Field.getMyRight().getDistance(robotPose.transformBy(turretToRobot).getTranslation());
    }
    return Field.getMyHub().getDistance((robotPose.transformBy(turretToRobot).getTranslation()));
  }

  public double getLookaheadDistanceToHub() {
    return Field.getMyHub()
        .getDistance(
            (DynamicShootingCalculator.getInstance()
                .getLookaheadPose()
                // .transformBy(turretToRobot)
                .getTranslation()));
  }
}
