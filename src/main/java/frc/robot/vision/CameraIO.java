package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface CameraIO {
  @AutoLog
  public static class CameraIOInputs {
    public PoseObservation[] results = new PoseObservation[0];
  }

  public static record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      double reprojError,
      int tagCount,
      double averageTagDistance) {}

  public default void updateInputs(CameraIOInputs inputs) {}
  ;
}
