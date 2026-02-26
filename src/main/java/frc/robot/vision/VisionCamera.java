package frc.robot.vision;

import edu.wpi.first.math.geometry.Transform3d;

public class VisionCamera {
  private final String name;
  private final Transform3d transform;

  public VisionCamera(String name, Transform3d transform) {
    this.name = name;
    this.transform = transform;
  }

  public String getName() {
    return name;
  }

  public Transform3d getTransform() {
    return transform;
  }
}
