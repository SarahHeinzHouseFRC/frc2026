package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.FieldConstants;

public class VisionConstants {
  // chat and I made ALL of these constants up
  public static final double LINEAR_STD_DEV_AT_ONE_METER = 1;
  public static final double ANGULAR_STD_DEV_AT_ONE_METER = 5;
  public static final double MIN_LINEAR_STD_DEV = 0.05;
  public static final double MAX_LINEAR_STD_DEV = 5.0;
  public static final double MIN_ANGULAR_STD_DEV = 0.1;
  public static final double MAX_ANGULAR_STD_DEV = 10.0;

  public static final double FIELD_BORDER_MARGIN_METERS = 0.25;
  public static final double MAX_Z = 0.6;
  public static final double MIN_Z = -0.3;
  public static final double MAX_TILT_RADIANS = .5;
  public static final double MAX_SINGLE_TAG_AMBIGUITY = 0.2;
  public static final double MAX_MULTI_TAG_REPROJECTION_ERROR = 1.0;
  public static final double MAX_OBSERVATION_AGE_SECONDS = 0.5;
  public static final double MAX_FUTURE_TIMESTAMP_SECONDS = 0.1;
  public static final double MAX_SINGLE_TAG_POSE_DIFFERENCE_METERS = 1.0;

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
