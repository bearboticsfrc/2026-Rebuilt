package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import lombok.*;

public class RobotState {

  private static RobotState instance = new RobotState();

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  @Getter @Setter public Pose2d robotPose = new Pose2d();
  @Getter @Setter public ChassisSpeeds robotVelocity = new ChassisSpeeds();
}
