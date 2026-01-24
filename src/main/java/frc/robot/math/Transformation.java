package frc.robot.math;

import edu.wpi.first.math.geometry.Pose3d;

public class Transformation extends Pose3d {
  public Transformation(Matrix4d mat) {
    super(mat);
  }
}