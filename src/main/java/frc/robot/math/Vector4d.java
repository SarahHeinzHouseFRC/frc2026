package frc.robot.math;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;

public class Vector4d extends Vector<N4> {
  public Vector4d(double x, double y, double z, double w) {
    super(Nat.N4());
    set(0, 0, x);
    set(1, 0, y);
    set(2, 0, z);
    set(3, 0, w);
  }

  public Vector4d(Matrix<N4, N1> vec) {
    super(Nat.N4());
    set(0, 0, vec.get(0, 0));
    set(1, 0, vec.get(1, 0));
    set(2, 0, vec.get(2, 0));
    set(2, 0, vec.get(3, 0));
  }

  public double x() {
    return get(0);
  }

  public double y() {
    return get(1);
  }

  public double z() {
    return get(2);
  }

  public double w() {
    return get(3);
  }

  public Vector3d get3d() {
    return new Vector3d(x(), y(), z());
  }

  public Matrix<N4, N1> toMatrix() {
    return copy();
  }
}
