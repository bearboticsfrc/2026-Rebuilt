package frc.robot.epilogue;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.Map;

@CustomLoggerFor(Map.class)
public class MapLogger extends ClassSpecificLogger<Map> {
  public MapLogger() {
    super(Map.class);
  }

  @Override
  public void update(EpilogueBackend backend, Map map) {
    for (Map.Entry<String, Pose2d> entry : ((Map<String, Pose2d>) map).entrySet()) {
      backend.log(entry.getKey(), entry.getValue(), Pose2d.struct);
    }
  }
}
