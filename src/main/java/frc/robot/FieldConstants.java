package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;

// https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf
public class FieldConstants {
  public static final FieldType TYPE = FieldType.WELDED;

  public enum FieldType {
    WELDED,
    ANDYMARK,
  }

  public static final Translation3d TOWER;
  public static final Translation3d TOWER_L1;
  public static final Translation3d TOWER_L2;
  public static final Translation3d TOWER_L3;
  public static final Translation3d DEPOT;
  public static final Translation3d HUB;
  public static final Translation3d TRENCH_L;
  public static final Translation3d TRENCH_R;

  public static final Translation3d SHOT_TARGET_L;
  public static final Translation3d SHOT_TARGET_R;

  static {
    if (TYPE == FieldType.WELDED) {
      TOWER = fromInches(0, 0, 0);
      TOWER_L1 = fromInches(0, 0, 0);
      TOWER_L2 = fromInches(0, 0, 0);
      TOWER_L3 = fromInches(0, 0, 0);
      DEPOT = fromInches(0, 0, 0);
      HUB = fromInches(182.11, 158.845, 72);
      TRENCH_L = fromInches(0, 0, 0);
      TRENCH_R = fromInches(0, 0, 0);
      SHOT_TARGET_R = fromInches(91, 79, 0);
      SHOT_TARGET_L = fromInches(91, 238, 0);
    } else if (TYPE == FieldType.ANDYMARK) {
      TOWER = fromInches(0, 0, 0);
      TOWER_L1 = fromInches(0, 0, 0);
      TOWER_L2 = fromInches(0, 0, 0);
      TOWER_L3 = fromInches(0, 0, 0);
      DEPOT = fromInches(0, 0, 0);
      HUB = fromInches(181.56, 158.32, 72);
      TRENCH_L = fromInches(0, 0, 0);
      TRENCH_R = fromInches(0, 0, 0);
      SHOT_TARGET_R = fromInches(91, 79, 0);
      SHOT_TARGET_L = fromInches(91, 238, 0);
    } else {
      throw new IllegalStateException("Invalid field type");
    }
  }

  private static Translation3d fromInches(double x, double y, double z) {
    return new Translation3d(x * 0.0254, y * 0.0254, z * 0.0254);
  }
}
