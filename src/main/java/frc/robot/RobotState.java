package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import bearlib.util.GeomUtil;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.rebuilt.Field;
import frc.robot.subsystems.shooter.DynamicShootingCalculator;
import java.util.function.BooleanSupplier;
import lombok.*;

public class RobotState {

  private static RobotState instance = new RobotState();

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  /** Turret translation. */
  @Logged public Translation2d turretTranslation;

  /** Turret pose. */
  @Logged @Getter public Pose2d turretPose;

  /** Turret transform. */
  public static final Transform2d turretToRobot =
      new Transform2d(Inches.of(-6.25), Inches.of(-6.25), new Rotation2d());

  /** Trigger for the {@code isStopped()} boolean. */
  @Logged public Trigger stopped = new Trigger(() -> isStopped());

  /** Signals whether or not to update the robot pose in auto. */
  public boolean updatePoseInAutonomous = true;

  /** Signals whether the robot is red alliance. */
  public boolean isRedAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red;
  }

  /** Signals whether the robot is blue alliance. */
  public boolean isBlueAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue;
  }

  @Logged @Getter @Setter public Pose2d robotPose = new Pose2d(); // robot pose2d.

  @Getter @Setter public ChassisSpeeds robotVelocity = new ChassisSpeeds(); // chassis speeds.

  @Logged @Getter @Setter public boolean shooting; // whether or not the robot is shooting.

  /** The rotation of the robot pose. */
  public Rotation2d getRotation() {
    return robotPose.getRotation();
  }

  /** Updates turret pose and robot pose. */
  public void updatePose() {
    turretPose = robotPose.transformBy(turretToRobot);
    turretTranslation = (robotPose.transformBy(turretToRobot).getTranslation());
  }

  /** The field velocity of the chassis. */
  @Logged
  public ChassisSpeeds getFieldVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(robotVelocity, getRotation());
  }

  /** Signals whether the robot is stopped. */
  public boolean isStopped() {
    if (isInAllianceZone()) {
      return Math.abs(getFieldVelocity().vxMetersPerSecond) < 0.00005
          && Math.abs(getFieldVelocity().vyMetersPerSecond) < 0.00005
          && Math.abs(getFieldVelocity().omegaRadiansPerSecond) < 0.00005;
    }
    return false;
  }

  /** Signals whether or not the robot is in the alliance zone. */
  @Logged
  public boolean isInAllianceZone() {
    if (isBlueAlliance() && robotPose.getX() < Field.getMyAllianceLine().getX()) return true;
    else if (isRedAlliance() && robotPose.getX() > Field.getMyAllianceLine().getX()) return true;
    else return false;
  }

  /** Signals whether or not the robot is in the neutral zone. */
  @Logged
  public BooleanSupplier isInNeutralZone() {
    if (isBlueAlliance() && robotPose.getX() > Field.getMyAllianceLine().getX()) return () -> true;
    else if (isRedAlliance() && robotPose.getX() < Field.getMyAllianceLine().getX())
      return () -> true;
    else return () -> false;
  }

  /** Signals whether or not the robot is field left. */
  @Logged
  public boolean iSLeft() {
    return (isBlueAlliance() && robotPose.getY() > Field.getMyAllianceLine().getY()
        || isRedAlliance() && robotPose.getY() < Field.getMyAllianceLine().getY());
  }

  /** Signals whether or not the robot is field left neutral zone. */
  @Logged
  public boolean isLeftNeutralZone() {
    return (iSLeft() && isInNeutralZone().getAsBoolean());
  }

  /** Signals whether or not the robot is field right neutral zone. */
  @Logged
  public boolean isRightNeutralZone() {
    return (!iSLeft() && isInNeutralZone().getAsBoolean());
  }

  /** Signals whether or not the robot is in a restricted shooting zone. */
  @Logged
  public boolean shootBlocked() {
    return GeomUtil.inZone(Field.getMyNet(), robotPose)
        || GeomUtil.inZone(Field.getMyTower(), robotPose);
  }

  /** The angle from the turret pose to the hub. */
  public Rotation2d getAngleToHub() {
    return Field.getMyHub()
        .minus((robotPose.transformBy(turretToRobot).getTranslation()))
        .getAngle()
        .plus(new Rotation2d(Degrees.of(180)));
  }

  /** The distance from the turret pose to the hub. */
  public double getDistanceToHub() {
    return Field.getMyHub().getDistance((robotPose.transformBy(turretToRobot).getTranslation()));
  }

  /** The distance from the turret pose to the shooting target. */
  @Logged
  public double getTargetDistance() {
    if (isInNeutralZone().getAsBoolean()) {
      return (iSLeft())
          ? Field.getMyLeft().getDistance((robotPose.transformBy(turretToRobot).getTranslation()))
          : Field.getMyRight().getDistance(robotPose.transformBy(turretToRobot).getTranslation());
    }
    return Field.getMyHub().getDistance((robotPose.transformBy(turretToRobot).getTranslation()));
  }

  /** The distance from the lookahead pose to the hub. */
  public double getLookaheadDistanceToHub() {
    return Field.getMyHub()
        .getDistance((DynamicShootingCalculator.getInstance().getLookaheadPose().getTranslation()));
  }

  /** The zone under the tower. */
  @Logged(name = "Tower")
  public Translation2d[] getTowerZone() {
    return Field.getMyTower().get();
  }

  /** The restricted shooting zone behind the hub. */
  @Logged(name = "Net")
  public Translation2d[] getNetZone() {
    return Field.getMyNet().get();
  }
}
