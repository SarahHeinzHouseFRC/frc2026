package frc.robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.FieldConstants;

public class VisionConstants {
  public static AprilTagFieldLayout aprilTagFieldLayout =
      AprilTagFieldLayout.loadField(
          switch (FieldConstants.TYPE) {
            case WELDED -> AprilTagFields.k2026RebuiltWelded;
            case ANDYMARK -> AprilTagFields.k2026RebuiltAndymark;
          });
  public static Pose3d redPose =
      switch (FieldConstants.TYPE) {
        case WELDED -> new Pose3d(
            new Translation3d(16.541, 8.069, 0), new Rotation3d(0, 0, Math.PI));
        case ANDYMARK -> new Pose3d(
            new Translation3d(16.513, 8.043, 0), new Rotation3d(0, 0, Math.PI));
      };
}
