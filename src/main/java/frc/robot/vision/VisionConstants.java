package frc.robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import frc.robot.FieldConstants;

public class VisionConstants {
  public static AprilTagFieldLayout aprilTagFieldLayout =
      AprilTagFieldLayout.loadField(
          switch (FieldConstants.TYPE) {
            case WELDED -> AprilTagFields.k2026RebuiltWelded;
            case ANDYMARK -> AprilTagFields.k2026RebuiltAndymark;
          });
}
