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

  public String getMyAlliance() {
    if (DriverStation.getAlliance().get() == Alliance.Blue) return "BLUE";
    else if (DriverStation.getAlliance().get() == Alliance.Red) return "RED";

    else return null;
  }

  @Getter @Setter public Pose2d robotPose = new Pose2d();
  @Getter @Setter public ChassisSpeeds robotVelocity = new ChassisSpeeds();

  @Logged
  public boolean isInAllianceZone() {

    if (getMyAlliance().equals("BLUE") && robotPose.getY() < Field.getMyAllianceLine().getY())
      return true;
    else if (getMyAlliance().equals("RED") && robotPose.getY() > Field.getMyAllianceLine().getY())
      return true;

    else return false;
    
  }

@Logged
public boolean isInNeutralZone() {

    if (getMyAlliance().equals("BLUE") && robotPose.getY() > Field.getMyAllianceLine().getY())
      return true;
    else if (getMyAlliance().equals("RED") && robotPose.getY() < Field.getMyAllianceLine().getY())
      return true;

    else return false;
  }

@Logged
public String getNeutralZoneDirection(){
  
  if (getMyAlliance().equals("BLUR") && isInNeutralZone() && robotPose.getX() > Field.getMyAllianceLine().getX()) return "L";
  else if (getMyAlliance().equals("BLUE") && isInNeutralZone() && robotPose.getX() < Field.getMyAllianceLine().getX()) return "R";

  else if (getMyAlliance().equals("RED") && isInNeutralZone() && robotPose.getX() < Field.getMyAllianceLine().getX()) return "L";
  else if (getMyAlliance().equals("RED") && isInNeutralZone() && robotPose.getX() > Field.getMyAllianceLine().getX()) return "R";

  else return null;
}  

}
