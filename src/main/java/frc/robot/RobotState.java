package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.field.Field;
import lombok.*;

public class RobotState {

  private static RobotState instance = new RobotState();

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  public boolean isRedAlliance() {
    return DriverStation.getAlliance().get() == Alliance.Red;
  }

  public boolean isBlueAlliance() {
    return DriverStation.getAlliance().get() == Alliance.Blue;
  }

  @Getter @Setter public Pose2d robotPose = new Pose2d();
  @Getter @Setter public ChassisSpeeds robotVelocity = new ChassisSpeeds();

  @Logged
  public boolean isInAllianceZone() {
    if (isBlueAlliance() && robotPose.getY() < Field.getMyAllianceLine().getY()) return true;
    else if (isBlueAlliance() && robotPose.getY() > Field.getMyAllianceLine().getY()) return true;
    else return false;
  }

  @Logged
  public boolean isInNeutralZone() {
    if (isBlueAlliance() && robotPose.getY() > Field.getMyAllianceLine().getY()) return true;
    else if (isRedAlliance() && robotPose.getY() < Field.getMyAllianceLine().getY()) return true;
    else return false;
  }

  @Logged
  public String getNeutralZoneDirection() {
    if (isBlueAlliance()
        && isInNeutralZone()
        && robotPose.getX() > Field.getMyAllianceLine().getX()) return "L";
    else if (isBlueAlliance()
        && isInNeutralZone()
        && robotPose.getX() < Field.getMyAllianceLine().getX()) return "R";
    else if (isRedAlliance()
        && isInNeutralZone()
        && robotPose.getX() < Field.getMyAllianceLine().getX()) return "L";
    else if (isRedAlliance()
        && isInNeutralZone()
        && robotPose.getX() > Field.getMyAllianceLine().getX()) return "R";
    else return null;
  }
}
