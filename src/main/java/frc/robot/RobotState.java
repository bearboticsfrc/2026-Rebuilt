package frc.robot;

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

  @Getter @Setter public Pose2d robotPose = new Pose2d();
  @Getter @Setter public ChassisSpeeds robotVelocity = new ChassisSpeeds();

  public int getMyAlliance() {
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      return 1;
    } else if (DriverStation.getAlliance().get() == Alliance.Red) {
    return -1;
  } else {
    return 0;
  }
}
  public boolean isInAllianceZone() {
   if (getMyAlliance() == 1 && robotPose.getTranslation().getY() < Field.getMyAllianceLine().getY()) {
    return true;
   } else if (getMyAlliance() == -1  && robotPose.getY() > Field.getMyAllianceLine().getY()) {
    return true;
   } else {
    return false;
   }

  }
}

