package frc.robot.math;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class Vector3d extends Vector<N3> {
  public Vector3d(double x, double y, double z) {
    super(Nat.N3());
    set(0, 0, x);
    set(1, 0, y);
    set(2, 0, z);
  }

  public Vector3d(Matrix<N3, N1> vec) {
    super(Nat.N3());
    set(0, 0, vec.get(0, 0));
    set(1, 0, vec.get(1, 0));
    set(2, 0, vec.get(2, 0));
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

  public Matrix<N3, N1> toMatrix() {
    return copy();
  }

  public double magnitude() {
    return Math.sqrt(x() * x() + y() * y() + z() * z());
  }
}
