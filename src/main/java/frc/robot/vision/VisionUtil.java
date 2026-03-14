package frc.robot.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.rebuilt.FieldHelpers;
import frc.robot.Robot;
import frc.robot.vision.VisionSystem.VisionEstimate;
import frc.spectrumLib.Telemetry;
import java.util.Comparator;
import java.util.List;

public class VisionUtil {
  private static final double MAX_TIME_DELTA_SECONDS = 0.1;
  private static final double LARGE_VARIANCE = 999999.0;

  /** Helper to integrate a single estimate */
  private void integrateSingleEstimate(VisionEstimate estimate) {
    if (estimate != null) {
      Robot.get()
          .getSwerve()
          .addVisionMeasurement(
              estimate.pose().toPose2d(), estimate.timestampSeconds(), estimate.stdDevs());
    }
  }

  /** Helper to integrate multiple estimates close in time by fusing them together first */
  public void integrateMultipleEstimates(List<VisionSystem.VisionEstimate> list) {
    // Collect non-null estimates
    // List<VisionEstimate> list = new ArrayList<>();
    // for (VisionEstimate e : estimates) {
    //   if (e != null) list.add(e);
    // }
    if (list.isEmpty()) return;

    // Sort by timestamp ascending (old -> new). fuseEstimates expects to project
    // older -> newer.
    list.sort(Comparator.comparingDouble(VisionEstimate::timestampSeconds));

    // Iteratively group/fuse estimates close in time.
    VisionEstimate currentGroup = list.get(0);
    for (int i = 1; i < list.size(); i++) {
      VisionEstimate next = list.get(i);
      double timeDelta = Math.abs(next.timestampSeconds() - currentGroup.timestampSeconds());

      if (timeDelta < MAX_TIME_DELTA_SECONDS) {
        // Fuse into current group (currentGroup older, next newer)
        currentGroup = fuseEstimates(currentGroup, next);
      } else {
        // No close timestamp: integrate current group and start a new one
        integrateSingleEstimate(currentGroup);
        currentGroup = next;
      }
    }
    // integrate the final fused group
    integrateSingleEstimate(currentGroup);
  }

  private boolean rejectionCheck(Pose2d pose, double targetSize) {
    /* rejections */
    if (FieldHelpers.poseOutOfField(pose)) {
      return true;
    }

    if (Math.abs(Robot.get().getSwerve().getCurrentRobotChassisSpeeds().omegaRadiansPerSecond)
        >= 1.6) {
      return true;
    }

    // Final check, if it's small reject, else return false and integrate
    return targetSize <= 0.025;
  }

  /**
   * Set robot pose to vision pose only if LL has good tag reading
   *
   * @return if the pose was accepted and integrated
   */
  public boolean resetPoseToVision(
      boolean targetInView, Pose3d botpose3D, Pose2d megaPose, double poseTimestamp) {

    boolean reject = false;
    if (targetInView) {
      // replace botpose with this.pose
      Pose2d botpose = botpose3D.toPose2d();
      Pose2d pose;

      // Check if the vision pose is bad and don't trust it
      if (FieldHelpers.poseOutOfField(botpose3D)) { // pose out of field
        Telemetry.log("Pose out of field", reject);
        reject = true;
      } else if (Math.abs(botpose3D.getZ()) > 0.25) { // when in air
        Telemetry.log("Pose in air", reject);
        reject = true;
      } else if ((Math.abs(botpose3D.getRotation().getX()) > 5
          || Math.abs(botpose3D.getRotation().getY()) > 5)) { // when tilted

        Telemetry.log("Pose tilted", reject);
        reject = true;
      }

      // don't continue
      if (reject) {
        return !reject; // return the success status
      }

      // Posts Current X,Y, and Angle (Theta) values
      double[] visionPose = {botpose.getX(), botpose.getY(), botpose.getRotation().getDegrees()};
      Telemetry.log("Current Vision Pose: ", visionPose);

      Robot.get()
          .getSwerve()
          .setVisionMeasurementStdDevs(VecBuilder.fill(0.00001, 0.00001, 0.00001));

      Pose2d integratedPose = new Pose2d(megaPose.getTranslation(), botpose.getRotation());
      Robot.get().getSwerve().addVisionMeasurement(integratedPose, poseTimestamp);
      pose = Robot.get().getSwerve().getRobotPose();
      // Gets updated pose of x, y, and theta values
      visionPose = new double[] {pose.getX(), pose.getY(), pose.getRotation().getDegrees()};
      Telemetry.log("Vision Pose Reset To: ", visionPose);

      // print "success"
      return true;
    }
    return false; // target not in view
  }

  /** Fuses two vision pose estimates using inverse-variance weighting. (FRC254 2025) */
  private VisionEstimate fuseEstimates(VisionEstimate a, VisionEstimate b) {
    // Ensure b is the newer measurement
    if (b.timestampSeconds() < a.timestampSeconds()) {
      VisionEstimate tmp = a;
      a = b;
      b = tmp;
    }

    // Project both estimates to the same timestamp using odometry
    Transform2d a_T_b =
        Robot.get()
            .getSwerve()
            .getPoseAtTimestamp(b.timestampSeconds())
            .minus(Robot.get().getSwerve().getPoseAtTimestamp(a.timestampSeconds()));

    Pose2d poseA = a.pose().toPose2d().transformBy(a_T_b);
    Pose2d poseB = b.pose().toPose2d();

    // Inverse‑variance weighting
    var varianceA = a.stdDevs().elementTimes(a.stdDevs());
    var varianceB = b.stdDevs().elementTimes(b.stdDevs());

    Rotation2d fusedHeading = poseB.getRotation();
    if (varianceA.get(2, 0) < LARGE_VARIANCE && varianceB.get(2, 0) < LARGE_VARIANCE) {
      fusedHeading =
          new Rotation2d(
              poseA.getRotation().getCos() / varianceA.get(2, 0)
                  + poseB.getRotation().getCos() / varianceB.get(2, 0),
              poseA.getRotation().getSin() / varianceA.get(2, 0)
                  + poseB.getRotation().getSin() / varianceB.get(2, 0));
    }

    double weightAx = 1.0 / varianceA.get(0, 0);
    double weightAy = 1.0 / varianceA.get(1, 0);
    double weightBx = 1.0 / varianceB.get(0, 0);
    double weightBy = 1.0 / varianceB.get(1, 0);

    Pose2d fusedPose =
        new Pose2d(
            new Translation2d(
                (poseA.getTranslation().getX() * weightAx
                        + poseB.getTranslation().getX() * weightBx)
                    / (weightAx + weightBx),
                (poseA.getTranslation().getY() * weightAy
                        + poseB.getTranslation().getY() * weightBy)
                    / (weightAy + weightBy)),
            fusedHeading);

    Matrix<N3, N1> fusedStdDev =
        VecBuilder.fill(
            Math.sqrt(1.0 / (weightAx + weightBx)),
            Math.sqrt(1.0 / (weightAy + weightBy)),
            Math.sqrt(1.0 / (1.0 / varianceA.get(2, 0) + 1.0 / varianceB.get(2, 0))));

    int numTags = a.numTags() + b.numTags();
    double time = b.timestampSeconds();

    return new VisionEstimate(new Pose3d(fusedPose), time, fusedStdDev, numTags);
  }
}
