package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;

// https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf
public class FieldConstants {
  public static final FieldType TYPE = FieldType.WELDED;

  public enum FieldType {
    WELDED,
    ANDYMARK,
  }

  public static final Translation3d HUB;

  public static final Translation3d SHOT_TARGET_L;
  public static final Translation3d SHOT_TARGET_R;

  public static final double fieldWidth;

  static {
    if (TYPE == FieldType.WELDED) {
      HUB = fromInches(182.11, 158.845, 72);
      SHOT_TARGET_R = fromInches(96, 119, 0);
      SHOT_TARGET_L = fromInches(96, 198, 0);
      fieldWidth = 8.06933;
    } else if (TYPE == FieldType.ANDYMARK) {
      HUB = fromInches(181.56, 158.32, 72);
      SHOT_TARGET_R = fromInches(96, 119, 0);
      SHOT_TARGET_L = fromInches(96, 198, 0);
      fieldWidth = 8.04266;
    } else {
      throw new IllegalStateException("Invalid field type");
    }
  }

  private static Translation3d fromInches(double x, double y, double z) {
    return new Translation3d(x * 0.0254, y * 0.0254, z * 0.0254);
  }
}
