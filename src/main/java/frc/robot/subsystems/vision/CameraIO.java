package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;

public interface CameraIO {
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

  default void setIsBlue(boolean blue) {}
}
